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
                blocks[2] = 0x0000 | (rds_params.ecc & 0x00FF); // Variant 0 (ECC)
                blocks[3] = 0x0000; // PIN = 0x0000
            } else {
                // Fallback to Group 0A
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }

        } else if(state == 10) { // Group 1A (LIC - Variant 3)
            if (rds_params.lic != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5); 
                blocks[2] = 0x3000 | (rds_params.lic & 0x00FF); // Variant 3 (LIC)
                blocks[3] = 0x0000; // PIN = 0x0000
            } else {
                // Fallback to Group 0A
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }
        } else if(state < 19) { // Group 15A (Long PS - 32 Bytes across 8 segments)
            int lps_seg = state - 11;
            blocks[1] = 0xF000 | (rds_params.tp << 10) | (rds_params.pty << 5) | (lps_seg & 0x07);
            blocks[2] = (uint8_t)rds_params.lps[lps_seg * 4 + 0] << 8 | (uint8_t)rds_params.lps[lps_seg * 4 + 1];
            blocks[3] = (uint8_t)rds_params.lps[lps_seg * 4 + 2] << 8 | (uint8_t)rds_params.lps[lps_seg * 4 + 3];
            lps_state++;
            if (lps_state >= 8) lps_state = 0;
        }
    
        state++;

        // Reset state sequencing
        int max_states = 19;
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
    copy_utf8_padded(rt, rds_params.rt, RT_LENGTH);
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
}/*
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

float carrier_57[] = {0.0, 1.0, 1.2246467991473532e-16, -1.0}; // sine wave at 57 kHz, 228 kHz sample rate[cite: 3]

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

uint16_t offset_words[] = {0x0FC, 0x198, 0x168, 0x1B4}; //[cite: 3]

/* Helper: Transcode standard UTF-8 characters to EBU Latin RDS Character Set */
static uint8_t utf8_to_ebu(const char **src_ptr) {
    const uint8_t *src = (const uint8_t *)*src_ptr;
    if (*src < 0x80) {
        uint8_t c = *src;
        (*src_ptr)++;
        return c;
    }

    // Two-byte UTF-8 sequences
    if ((src[0] & 0xE0) == 0xC0 && src[1]) {
        uint32_t codepoint = ((src[0] & 0x1F) << 6) | (src[1] & 0x3F);
        *src_ptr += 2;
        switch (codepoint) {
            case 0x00E0: return 0x85; // à
            case 0x00E1: return 0x81; // á
            case 0x00E2: return 0x83; // â
            case 0x00E4: return 0x8B; // ä
            case 0x00E7: return 0x8D; // ç
            case 0x00E8: return 0x95; // è
            case 0x00E9: return 0x91; // é
            case 0x00EA: return 0x93; // ê
            case 0x00EB: return 0x9B; // ë
            case 0x00EC: return 0x9D; // ì
            case 0x00ED: return 0x99; // í
            case 0x00EE: return 0x9B; // î
            case 0x00EF: return 0x9F; // ï
            case 0x00F1: return 0x8E; // ñ
            case 0x00F2: return 0xA5; // ò
            case 0x00F3: return 0xA1; // ó
            case 0x00F4: return 0xA3; // ô
            case 0x00F6: return 0xAB; // ö
            case 0x00F9: return 0xB5; // ù
            case 0x00FA: return 0xB1; // ú
            case 0x00FB: return 0xB3; // û
            case 0x00FC: return 0xBB; // ü
            case 0x00C4: return 0x8A; // Ä
            case 0x00D6: return 0xAA; // Ö
            case 0x00DC: return 0xBA; // Ü
            case 0x00DF: return 0x1E; // ß
            default: return '?';
        }
    }

    (*src_ptr)++;
    return '?';
}

static void convert_string_to_ebu(const char *in, char *out, size_t max_len) {
    const char *p = in;
    size_t count = 0;
    while (*p && count < max_len) {
        out[count++] = (char)utf8_to_ebu(&p);
    }
    while (count < max_len) {
        out[count++] = ' ';
    }
}

/* Classical CRC computation */
uint16_t crc(uint16_t block) { //[cite: 3]
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
int get_rds_ct_group(uint16_t *blocks, int enabled) { //[cite: 3]
    static int latest_minutes = -1;

    time_t now;
    struct tm *utc;
    
    now = time (NULL);
    utc = gmtime (&now);

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
    } else return 0;
}

/* Creates an RDS group. */
void get_rds_group(int *buffer, int stereo, int ct_clock_enabled) { //[cite: 3]
    static int state = 0;
    static int ps_state = 0;
    static int lps_state = 0;
    static int rt_state = 0;
    static int ptyn_state = 0;
    static int af_state = 0;
    uint16_t blocks[GROUP_LENGTH] = {rds_params.pi, 0, 0, 0}; //[cite: 3]
    
    // Generate block content
    if(!get_rds_ct_group(blocks, ct_clock_enabled)) { // CT group has priority[cite: 3]
        if(state < 4) { // Group 0A (PS, AF, DI, TA)[cite: 3]
            blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state; //[cite: 3]
            blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2; //[cite: 3]
            if(rds_params.af[0]) { // AF[cite: 3]
                if(af_state == 0) { 
                    blocks[2] = (rds_params.af[0] + 224) << 8 | rds_params.af[1]; //[cite: 3]
                } else {
                    if(rds_params.af[af_state+1]) {
                        blocks[2] = rds_params.af[af_state] << 8 | rds_params.af[af_state+1]; //[cite: 3]
                    } else {
                        blocks[2] = rds_params.af[af_state] << 8 | 0xCD; //[cite: 3]
                    }
                }
                af_state = af_state + 2;
                if(af_state > rds_params.af[0]) af_state = 0;
            } else {
                blocks[2] = 224 << 8 | 0xCD; //[cite: 3]
            }
            blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
            ps_state++;
            if(ps_state >= 4) ps_state = 0;

        } else if(state < 8) { // Group 2A (Radiotext)[cite: 3]
            blocks[1] = 0x2000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ab << 4 | rt_state; //[cite: 3]
            blocks[2] = (uint8_t)rds_params.rt[rt_state*4+0] << 8 | (uint8_t)rds_params.rt[rt_state*4+1];
            blocks[3] = (uint8_t)rds_params.rt[rt_state*4+2] << 8 | (uint8_t)rds_params.rt[rt_state*4+3];
            rt_state++;
            if(rt_state >= 16) rt_state = 0;

        } else if(state == 8) { // Group 10A (PTYN)[cite: 3]
            blocks[1] = 0xA000 | (rds_params.tp << 10) | (rds_params.pty << 5) | (ptyn_state & 0x03); //[cite: 3]
            blocks[2] = (uint8_t)rds_params.ptyn[ptyn_state * 4] << 8 | (uint8_t)rds_params.ptyn[ptyn_state * 4 + 1];
            blocks[3] = (uint8_t)rds_params.ptyn[ptyn_state * 4 + 2] << 8 | (uint8_t)rds_params.ptyn[ptyn_state * 4 + 3];

            ptyn_state++;
            if(ptyn_state >= 2) ptyn_state = 0;

        } else if(state == 9) { // Group 1A (ECC - Variant 0)[cite: 3]
            if (rds_params.ecc != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5);  //[cite: 3]
                blocks[2] = 0x0000 | (rds_params.ecc & 0x00FF); // Variant 0 (ECC)[cite: 3]
                blocks[3] = 0x0000; // PIN = 0x0000[cite: 3]
            } else {
                // Correct Fallback to Group 0A
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }

        } else if(state == 10) { // Group 1A (LIC - Variant 3)[cite: 3]
            if (rds_params.lic != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5);  //[cite: 3]
                blocks[2] = 0x3000 | (rds_params.lic & 0x00FF); // Variant 3 (LIC)[cite: 3]
                blocks[3] = 0x0000; // PIN = 0x0000[cite: 3]
            } else {
                // Correct Fallback to Group 0A
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3 | ps_state;
                blocks[1] |= ((rds_params.di >> (3 - ps_state)) & 0x01) << 2;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = (uint8_t)rds_params.ps[ps_state*2] << 8 | (uint8_t)rds_params.ps[ps_state*2+1];
                ps_state = (ps_state + 1) % 4;
            }
        } else if(state < 19) { // Group 15A (Long PS - 32 Bytes across 8 segments)
            int lps_seg = state - 11;
            blocks[1] = 0xF000 | (rds_params.tp << 10) | (rds_params.pty << 5) | (lps_seg & 0x07);
            blocks[2] = (uint8_t)rds_params.lps[lps_seg * 4 + 0] << 8 | (uint8_t)rds_params.lps[lps_seg * 4 + 1];
            blocks[3] = (uint8_t)rds_params.lps[lps_seg * 4 + 2] << 8 | (uint8_t)rds_params.lps[lps_seg * 4 + 3];
            lps_state++;
            if (lps_state >= 8) lps_state = 0;
        }
    
        state++;

        // Reset state sequencing based on active optional parameters
        int max_states = 19;
        if(state >= max_states)
            state = 0;
    }
    
    // Calculate the checkword for each block and emit the bits
    for(int i=0; i<GROUP_LENGTH; i++) { //[cite: 3]
        uint16_t block = blocks[i]; //[cite: 3]
        uint16_t check = crc(block) ^ offset_words[i]; //[cite: 3]
        for(int j=0; j<BLOCK_SIZE; j++) { //[cite: 3]
            *buffer++ = ((block & (1<<(BLOCK_SIZE-1))) != 0); //[cite: 3]
            block <<= 1; //[cite: 3]
        }
        for(int j=0; j<POLY_DEG; j++) { //[cite: 3]
            *buffer++= ((check & (1<<(POLY_DEG-1))) != 0); //[cite: 3]
            check <<= 1; //[cite: 3]
        }
    }
}

void get_rds_samples(float *buffer, int count, int stereo, int ct_clock_enabled, float sample_volume) { //[cite: 3]
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
                get_rds_group(bit_buffer, stereo, ct_clock_enabled); //[cite: 3]
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

void set_rds_pi(uint16_t pi_code) { //[cite: 3]
    rds_params.pi = pi_code;
}

void set_rds_rt(char *rt) { //[cite: 3]
    convert_string_to_ebu(rt, rds_params.rt, RT_LENGTH);
}

void set_rds_ps(char *ps) { //[cite: 3]
    convert_string_to_ebu(ps, rds_params.ps, PS_LENGTH);
}

void set_rds_lps(char *lps) {
    convert_string_to_ebu(lps, rds_params.lps, LPS_LENGTH);
}

void set_rds_ptyn(char *ptyn) { //[cite: 3]
    convert_string_to_ebu(ptyn, rds_params.ptyn, 8);
    rds_params.ptyn[8] = 0;
}

void set_rds_af(int *af_array) { //[cite: 3]
    rds_params.af[0] = af_array[0];
    int f;
    for(f = 1; f < af_array[0]+1; f++) {
        rds_params.af[f] = af_array[f];
    }
}

void set_rds_pty(int pty) { //[cite: 3]
    rds_params.pty = pty;
}

void set_rds_di(int di) { //[cite: 3]
    rds_params.di = di;
}

void set_rds_ta(int ta) { //[cite: 3]
    rds_params.ta = ta;
}

void set_rds_tp(int tp) { //[cite: 3]
    rds_params.tp = tp;
}

void set_rds_ms(int ms) { //[cite: 3]
    rds_params.ms = ms;
}

void set_rds_ab(int ab) { //[cite: 3]
    rds_params.ab = ab;
}

void set_rds_ecc(uint16_t ecc) { //[cite: 3]
    rds_params.ecc = ecc;
}

void set_rds_lic(uint16_t lic) { //[cite: 3]
    rds_params.lic = lic;
}/*
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
    char rt[RT_LENGTH];
    char ptyn[9];
    int af[100];
    int di;
} rds_params = { 0 };

uint16_t offset_words[] = {0x0FC, 0x198, 0x168, 0x1B4};

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
    
    now = time (NULL);
    utc = gmtime (&now);

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
    } else return 0;
}

/* Creates an RDS group. */
void get_rds_group(int *buffer, int stereo, int ct_clock_enabled) {
    static int state = 0;
    static int ps_state = 0;
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
            blocks[3] = rds_params.ps[ps_state*2] << 8 | rds_params.ps[ps_state*2+1];
            ps_state++;
            if(ps_state >= 4) ps_state = 0;

        } else if(state < 8) { // Group 2A (Radiotext)
            blocks[1] = 0x2000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ab << 4 | rt_state;
            blocks[2] = rds_params.rt[rt_state*4+0] << 8 | rds_params.rt[rt_state*4+1];
            blocks[3] = rds_params.rt[rt_state*4+2] << 8 | rds_params.rt[rt_state*4+3];
            rt_state++;
            if(rt_state >= 16) rt_state = 0;

        } else if(state == 8) { // Group 10A (PTYN)
            blocks[1] = 0xA000 | (rds_params.tp << 10) | (rds_params.pty << 5) | (ptyn_state & 0x03);
            blocks[2] = rds_params.ptyn[ptyn_state * 4] << 8 | rds_params.ptyn[ptyn_state * 4 + 1];
            blocks[3] = rds_params.ptyn[ptyn_state * 4 + 2] << 8 | rds_params.ptyn[ptyn_state * 4 + 3];

            ptyn_state++;
            if(ptyn_state >= 2) ptyn_state = 0;

        } else if(state == 9) { // Group 1A (ECC - Variant 0)
            if (rds_params.ecc != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5); 
                blocks[2] = 0x0000 | (rds_params.ecc & 0x00FF); // Variant 0 (ECC): 0x0000 + ECC payload
                blocks[3] = 0x0000;                              // Block D: PIN (No PIN = 0x0000)
            } else {
                // Fallback to Group 0A
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = rds_params.ps[0] << 8 | rds_params.ps[1];
            }

        } else if(state == 10) { // Group 1A (LIC - Variant 3)
            if (rds_params.lic != 0) {
                blocks[1] = 0x1000 | (rds_params.tp << 10) | (rds_params.pty << 5); 
                blocks[2] = 0x3000 | (rds_params.lic & 0x00FF); // Variant 3 (LIC): 0x3000 + LIC payload
                blocks[3] = 0x0000;                              // Block D: PIN (No PIN = 0x0000)
            } else {
                // Fallback to Group 0A
                blocks[1] = 0x0000 | rds_params.tp << 10 | rds_params.pty << 5 | rds_params.ta << 4 | rds_params.ms << 3;
                blocks[2] = 224 << 8 | 0xCD;
                blocks[3] = rds_params.ps[0] << 8 | rds_params.ps[1];
            }
        }
    
        state++;

        // Reset state sequencing based on active optional parameters
        int max_states = 9;
        if(rds_params.ecc != 0) max_states = 10;
        if(rds_params.lic != 0) max_states = 11;

        if(state >= max_states)
            state = 0;
    }
    
    // Calculate the checkword for each block and emit the bits
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
    strncpy(rds_params.rt, rt, 64);
    for(int i=0; i<64; i++) {
        if(rds_params.rt[i] == 0) rds_params.rt[i] = 32;
    }
}

void set_rds_ps(char *ps) {
    strncpy(rds_params.ps, ps, 8);
    for(int i=0; i<8; i++) {
        if(rds_params.ps[i] == 0) rds_params.ps[i] = 32;
    }
}

void set_rds_ptyn(char *ptyn) {
    strncpy(rds_params.ptyn, ptyn, 8);

    for(int i = 0; i < 8; i++) {
        if(rds_params.ptyn[i] == 0)
            rds_params.ptyn[i] = ' ';
    }
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
