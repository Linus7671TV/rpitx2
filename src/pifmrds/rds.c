/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2014 Christophe Jacquet, F8FTK
    
    See https://github.com/ChristopheJacquet/PiFmRds

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "waveforms.h"
#include "rds.h"

float carrier_57[] = {0.0, 1.0, 1.2246467991473532e-16, -1.0}; // sine wave at 57 kHz, 228 kHz sample rate

struct {
    uint16_t pi;
    uint16_t ecc;
    uint16_t lic;
    int ta;
    int pty;
    int tp;
    int ms;
    int ab;
    char ps[PS_LENGTH];
    char lps[LPS_LENGTH];
    char rt[RT_LENGTH];
    char ptyn[9];
    int af[100];
    int di;
} rds_params = { 0 };

uint16_t offset_words[] = {0x0FC, 0x198, 0x168, 0x1B4};

// RT+ State Variables
int rtplus_enabled = 0;
int rt_tag1_type = 1; // 1 = Item.Title
int rt_tag1_start = 0;
int rt_tag1_len = 0;
int rt_tag2_type = 4; // 4 = Item.Artist
int rt_tag2_start = 0;
int rt_tag2_len = 0;

/* Passes raw UTF-8 bytes directly into the buffer, padded with spaces */
static void copy_utf8_padded(const char *in, char *out, size_t max_len) {
    size_t i = 0;
    if (in) {
        while (in[i] != '\0' && i < max_len) {
            out[i] = in[i];
            i++;
        }
    }
    while (i < max_len) {
        out[i++] = ' ';
    }
}

/* Classical CRC computation */
uint16_t crc(uint16_t block) {
    uint16_t crc = 0;

    for(int j=0; j<BLOCK_SIZE; j++) {
        int bit = (block & MSB_BIT) != 0;
        block <<= 1;

        int msb = (crc >> (POLY_DEG-1)) & 1;
        crc <<= 1;
        if((msb ^ bit) != 0) {
            crc = crc ^ POLY;
        }
    }
    return crc;
}

/* Possibly generates a CT (clock time) group if the minute has just changed */
int get_rds_ct_group(uint16_t *blocks, int enabled) {
    static int latest_minutes = -1;

    time_t now;
    struct tm *utc;
    
    now = time(NULL);
    utc = gmtime(&now);

    if(!enabled) {
        latest_minutes = utc->tm_min;
        return 0;
    }
    if(utc->tm_min != latest_minutes) {
        latest_minutes = utc->tm_min;
        
        int l = utc->tm_mon <= 1 ? 1 : 0;
        int mjd = 14956 + utc->tm_mday + 
                        (int)((utc->tm_year - l) * 365.25) +
                        (int)((utc->tm_mon + 2 + l*12) * 30.6001);
        
        blocks[1] = 0x4400 | rds_params.tp << 10 | rds_params.pty << 5 | (mjd>>15);
        blocks[2] = (mjd<<1) | (utc->tm_hour>>4);
        blocks[3] = (utc->tm_hour & 0xF)<<12 | utc->tm_min<<6;
        
        utc = localtime(&now);
        
        int offset = utc->tm_gmtoff / (30 * 60);
        blocks[3] |= abs(offset);
        if(offset < 0) blocks[3] |= 0x20;
        
        return 1;
    }
    
    return 0;
}

/* Creates an RDS group. */
void get_rds_group(int *buffer, int stereo, int ct_clock_enabled) {
    static int state = 0;
    static int ps_state = 0;
    static int lps_state = 0;
    static int rt_state = 0;
    static int ptyn_state = 0;
    static int af_state = 0;
    uint16_t blocks[GROUP_LENGTH] = {rds_params.pi, 0, 0, 0};
    
    // Generate block content
    if(!get_rds_ct_group(blocks, ct_clock_enabled)) { // CT group has priority
        if(state < 4) { // Group 0A (PS, AF, DI, TA)
            blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
            blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
            if(rds_params.af[0]) { // AF
                if(af_state == 0) { 
                    blocks[2] = (rds_params.af[0] + 224) << 8 | rds_params.af[1];
                } else {
                    if(rds_params.af[af_state+1]) {
                        blocks[2] = rds_params.af[af_state] << 8 | rds_params.af[af_state+1];
                    } else {
                        blocks[2] = rds_params.af[af_state] << 8 | 0xCD;
                    }
                }
                af_state = af_state + 2;
                if(af_state > rds_params.af[0]) af_state = 0;
            } else {
                blocks[2] = 224 << 8 | 0xCD;
            }
            blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
            ps_state++;
            if(ps_state >= 4) ps_state = 0;

        } else if(state < 8) { // Group 2A (Radiotext)
            blocks[1] = 0x2000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ab << 4 | rt_state;
            blocks[2] = (uint8_t)rds_params.rt[rt_state*4+0] << 8 | (uint8_t)rds_params.rt[rt_state*4+1];
            blocks[3] = (uint8_t)rds_params.rt[rt_state*4+2] << 8 | (uint8_t)rds_params.rt[rt_state*4+3];
            rt_state++;
            if(rt_state >= 16) rt_state = 0;

        } else if(state == 8) { // Group 10A (PTYN)
            blocks[1] = 0xA000 | (rds_params.tp << 10) | (rds_params.pty << 5) | (ptyn_state & 0x03);
            blocks[2] = (uint8_t)rds_params.ptyn[ptyn_state * 4] << 8 | (uint8_t)rds_params.ptyn[ptyn_state * 4 + 1];
            blocks[3] = (uint8_t)rds_params.ptyn[ptyn_state * 4 + 2] << 8 | (uint8_t)rds_params.ptyn[ptyn_state * 4 + 3];

            ptyn_state++;
            if(ptyn_state >= 2) ptyn_state = 0;

        } else if(state == 9) { // Group 1A (ECC - Variant 0)
            if (rds_params.ecc != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5); 
                blocks[2] = 0x0000 | (rds_params.ecc & 0x00FF); 
                blocks[3] = 0x0000; 
            } else {
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }

        } else if(state == 10) { // Group 1A (LIC - Variant 3)
            if (rds_params.lic != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5); 
                blocks[2] = 0x3000 | (rds_params.lic & 0x00FF); 
                blocks[3] = 0x0000; 
            } else {
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }
            
        } else if(state < 19) { // Group 15A (Long PS)
            int lps_seg = state - 11;
            blocks[1] = 0xF000 | (rds_params.tp << 10) | (rds_params.pty << 5) | (lps_seg & 0x07);
            blocks[2] = (uint8_t)rds_params.lps[lps_seg * 4 + 0] << 8 | (uint8_t)rds_params.lps[lps_seg * 4 + 1];
            blocks[3] = (uint8_t)rds_params.lps[lps_seg * 4 + 2] << 8 | (uint8_t)rds_params.lps[lps_seg * 4 + 3];
            lps_state++;
            if (lps_state >= 8) lps_state = 0;
            
        } else if(state == 19) { // Group 3A (RT+ ODA Announcement)
            if (rtplus_enabled) {
                blocks[1] = 0x3000 | (rds_params.tp << 10) | (rds_params.pty << 5) | 0x16; // 0x16 = Map RT+ to Group 11A
                blocks[2] = 0x0000;
                blocks[3] = 0x4BD7; // AID for RT+
            } else {
                // Fallback padding
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }
            
        } else if(state == 20) { // Group 11A (RT+ Tags Payload)
            if (rtplus_enabled) {
                uint16_t b2_lower = 0;
                b2_lower |= (rds_params.ab & 1) << 4;   // CB toggle perfectly synced with standard RadioText AB toggle
                b2_lower |= (1) << 3;                   // Item Running bit set to active
                b2_lower |= (rt_tag1_type >> 3) & 0x07; // CT1 MSBs
                
                blocks[1] = 0xB000 | (rds_params.tp << 10) | (rds_params.pty << 5) | b2_lower;
                
                blocks[2] = ((rt_tag1_type & 0x07) << 13) | ((rt_tag1_start & 0x3F) << 7) | ((rt_tag1_len & 0x3F) << 1) | ((rt_tag2_type >> 5) & 0x01);
                
                blocks[3] = ((rt_tag2_type & 0x1F) << 11) | ((rt_tag2_start & 0x3F) << 5) | (rt_tag2_len & 0x1F);
            } else {
                // Fallback padding
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }
        }
    
        state++;

        // Reset state sequencing over larger boundary
        int max_states = 21;
        if(state >= max_states)
            state = 0;
    }
    
    // Calculate the checkword for each block and emit bits
    for(int i=0; i<GROUP_LENGTH; i++) {
        uint16_t block = blocks[i];
        uint16_t check = crc(block) ^ offset_words[i];
        for(int j=0; j<BLOCK_SIZE; j++) {
            *buffer++ = ((block & (1<<(BLOCK_SIZE-1))) != 0);
            block <<= 1;
        }
        for(int j=0; j<POLY_DEG; j++) {
            *buffer++= ((check & (1<<(POLY_DEG-1))) != 0);
            check <<= 1;
        }
    }
}

void get_rds_samples(float *buffer, int count, int stereo, int ct_clock_enabled, float sample_volume) {
    static int bit_buffer[BITS_PER_GROUP];
    static int bit_pos = BITS_PER_GROUP;
    static float sample_buffer[SAMPLE_BUFFER_SIZE] = {0};
    
    static int prev_output = 0;
    static int cur_output = 0;
    static int cur_bit = 0;
    static int sample_count = SAMPLES_PER_BIT;
    static int inverting = 0;
    static int phase = 0;

    static int in_sample_index = 0;
    static int out_sample_index = SAMPLE_BUFFER_SIZE-1;
        
    for(int i=0; i<count; i++) {
        if(sample_count >= SAMPLES_PER_BIT) {
            if(bit_pos >= BITS_PER_GROUP) {
                get_rds_group(bit_buffer, stereo, ct_clock_enabled);
                bit_pos = 0;
            }
            
            cur_bit = bit_buffer[bit_pos];
            prev_output = cur_output;
            cur_output = prev_output ^ cur_bit;
            
            inverting = (cur_output == 1);

            float *src = waveform_biphase;
            int idx = in_sample_index;

            for(int j=0; j<FILTER_SIZE; j++) {
                double val = (*src++);
                if(inverting) val = -val;
                sample_buffer[idx++] += val;
                if(idx >= SAMPLE_BUFFER_SIZE) idx = 0;
            }

            in_sample_index += SAMPLES_PER_BIT;
            if(in_sample_index >= SAMPLE_BUFFER_SIZE) in_sample_index -= SAMPLE_BUFFER_SIZE;
            
            bit_pos++;
            sample_count = 0;
        }
        
        double sample = sample_buffer[out_sample_index];
        sample_buffer[out_sample_index] = 0;
        out_sample_index++;
        if(out_sample_index >= SAMPLE_BUFFER_SIZE) out_sample_index = 0;
        
        sample = sample * carrier_57[phase];
        phase++;
        if(phase >= 4) phase = 0;
        
        *buffer++ = (sample * sample_volume);
        sample_count++;
    }
}

void set_rds_pi(uint16_t pi_code) {
    rds_params.pi = pi_code;
}

void set_rds_rt(char *rt) {
    // 1. Store clean padded ASCII to traditional RT buffer
    copy_utf8_padded(rt, rds_params.rt, RT_LENGTH);
    
    // 2. Parse text automatically to extract RT+ semantic boundaries
    char temp_rt[65];
    int rt_len = 0;
    while(rt[rt_len] != '\0' && rt_len < 64) {
        temp_rt[rt_len] = rt[rt_len];
        rt_len++;
    }
    temp_rt[rt_len] = '\0';
    
    char *dash = strstr(temp_rt, " - ");
    if (dash != NULL) {
        
        // Define Tag 2: Artist (Appears before the hyphen)
        rt_tag2_type = 4; // Code 4 = Item.Artist
        rt_tag2_start = 0;
        int artist_len = dash - temp_rt;
        if (artist_len > 32) artist_len = 32; // Tag 2 length payload is strictly 5-bits max
        
        // Define Tag 1: Title (Appears after the hyphen)
        rt_tag1_type = 1; // Code 1 = Item.Title
        rt_tag1_start = (dash - temp_rt) + 3;
        
        // Strip out trailing (Year) parenthesis from Title if it exists
        char *paren = strstr(dash + 3, " (");
        int title_len = 0;
        if (paren != NULL) {
            title_len = paren - (dash + 3);
        } else {
            title_len = strlen(dash + 3);
        }
        
        // Sanity check length bounding
        if (rt_tag1_start + title_len > 64) {
            title_len = 64 - rt_tag1_start;
        }
        
        if (artist_len > 0 && title_len > 0) {
            rt_tag2_len = artist_len - 1; // Standard requires (Length - 1)
            rt_tag1_len = title_len - 1;
            rtplus_enabled = 1;           // Fire up 3A and 11A groupings!
        } else {
            rtplus_enabled = 0;
        }
    } else {
        // Automatically kill RT+ payload if broadcasting single non-hyphenated strings
        rtplus_enabled = 0;
    }
}

void set_rds_ps(char *ps) {
    copy_utf8_padded(ps, rds_params.ps, PS_LENGTH);
}

void set_rds_lps(char *lps) {
    copy_utf8_padded(lps, rds_params.lps, LPS_LENGTH);
}

void set_rds_ptyn(char *ptyn) {
    copy_utf8_padded(ptyn, rds_params.ptyn, 8);
    rds_params.ptyn[8] = 0;
}

void set_rds_af(int *af_array) {
    rds_params.af[0] = af_array[0];
    int f;
    for(f = 1; f < af_array[0]+1; f++) {
        rds_params.af[f] = af_array[f];
    }
}

void set_rds_pty(int pty) {
    rds_params.pty = pty;
}

void set_rds_di(int di) {
    rds_params.di = di;
}

void set_rds_ta(int ta) {
    rds_params.ta = ta;
}

void set_rds_tp(int tp) {
    rds_params.tp = tp;
}

void set_rds_ms(int ms) {
    rds_params.ms = ms;
}

void set_rds_ab(int ab) {
    rds_params.ab = ab;
}

void set_rds_ecc(uint16_t ecc) {
    rds_params.ecc = ecc;
}

void set_rds_lic(uint16_t lic) {
    rds_params.lic = lic;
}
