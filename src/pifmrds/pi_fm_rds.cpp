#include <stdio.h> //[cite: 5]
#include <stdlib.h> //[cite: 5]
#include <unistd.h> //[cite: 5]
#include <string.h> //[cite: 5]
#include <errno.h> //[cite: 5]
#include <stdarg.h> //[cite: 5]
#include <stdint.h> //[cite: 5]
#include <math.h> //[cite: 5]
#include <time.h> //[cite: 5]
#include <signal.h> //[cite: 5]
#include <sys/types.h> //[cite: 5]
#include <sys/stat.h> //[cite: 5]
#include <fcntl.h> //[cite: 5]
#include <sys/mman.h> //[cite: 5]
#include <sndfile.h> //[cite: 5]
extern "C"
{
#include "rds.h" //[cite: 5]
#include "fm_mpx.h" //[cite: 5]
#include "control_pipe.h" //[cite: 5]
}
#include <librpitx/librpitx.h> //[cite: 5]
ngfmdmasync *fmmod; //[cite: 5]
#define DATA_SIZE 5000 //[cite: 5]
static void terminate(int num) //[cite: 5]
{
    delete fmmod; //[cite: 5]
    fm_mpx_close(); //[cite: 5]
    close_control_pipe(); //[cite: 5]
    exit(num); //[cite: 5]
}

static void fatal(char *fmt, ...) //[cite: 5]
{
    va_list ap; //[cite: 5]
    va_start(ap, fmt); //[cite: 5]
    vfprintf(stderr, fmt, ap); //[cite: 5]
    va_end(ap); //[cite: 5]
    terminate(0); //[cite: 5]
}

typedef struct tx_data {
    uint32_t carrier_freq; //[cite: 5]
    char *audio_file; //[cite: 5]
    uint16_t pi; //[cite: 5]
    uint16_t ecc; //[cite: 5]
    uint16_t lic; //[cite: 5]
    char *ps; //[cite: 5]
    char *lps;
    char *rt; //[cite: 5]
    char *ptyn; //[cite: 5]
    char *control_pipe; //[cite: 5]
    uint8_t pty; //[cite: 5]
    int af_array[100]; //[cite: 5]
    uint8_t raw; //[cite: 5]
    uint8_t drds; //[cite: 5]
    float preemp; //[cite: 5]
    int power; //[cite: 5]
    int rawSampleRate; //[cite: 5]
    int rawChannels; //[cite: 5]
    int deviation; //[cite: 5]
    uint8_t ta; //[cite: 5]
    uint8_t tp; //[cite: 5]
    float cutoff_freq; //[cite: 5]
    float audio_gain; //[cite: 5]
    float compressor_decay; //[cite: 5]
    float compressor_attack; //[cite: 5]
    float compressor_max_gain_recip; //[cite: 5]
    uint8_t enablecompressor; //[cite: 5]
    uint8_t rds_ct_enabled; //[cite: 5]
    float rds_volume; //[cite: 5]
    uint8_t disablestereo; //[cite: 5]
    uint8_t log; //[cite: 5]
    float limiter_threshold; //[cite: 5]
} tx_data; //[cite: 5]

int tx(tx_data *data) { //[cite: 5]
    struct sigaction sa; //[cite: 5]
    memset(&sa, 0, sizeof(sa)); //[cite: 5]
    sa.sa_handler = terminate; //[cite: 5]
    sigaction(SIGTERM, &sa, NULL); //[cite: 5]
    sigaction(SIGINT, &sa, NULL); //[cite: 5]
    sigaction(SIGQUIT, &sa, NULL); //[cite: 5]
    sigaction(SIGKILL, &sa, NULL); //[cite: 5]
    sigaction(SIGHUP, &sa, NULL); //[cite: 5]
    sigaction(SIGPWR, &sa, NULL); //[cite: 5]
    sigaction(SIGTSTP, &sa, NULL); //[cite: 5]
    sigaction(SIGSEGV, &sa, NULL); //[cite: 5]
    
    // Data structures for baseband data
    float audio_data[DATA_SIZE]; //[cite: 5]
    float devfreq[DATA_SIZE]; //[cite: 5]
    int data_len = 0; //[cite: 5]

    int generate_multiplex = 1; //[cite: 5]
    int dstereo = data->disablestereo; //[cite: 5]
    int drds = data->drds; //[cite: 5]
    float audio_gain = data->audio_gain; //[cite: 5]
    float compressor_decay = data->compressor_decay; //[cite: 5]
    float compressor_attack = data->compressor_attack; //[cite: 5]
    float compressor_max_gain_recip = data->compressor_max_gain_recip; //[cite: 5]
    int enablecompressor = data->enablecompressor; //[cite: 5]
    int rds_ct_enabled = data->rds_ct_enabled; //[cite: 5]
    float rds_volume = data->rds_volume; //[cite: 5]
    float limiter_threshold = data->limiter_threshold; //[cite: 5]

    // Set hardware power output level
    padgpio gpiopad; //[cite: 5]
    gpiopad.setlevel(data->power); //[cite: 5]

    // Initialize the baseband generator
    if(fm_mpx_open(data->audio_file, DATA_SIZE, data->raw, data->preemp, data->rawSampleRate, data->rawChannels, data->cutoff_freq) < 0) return 1; //[cite: 5]

    // Initialize the RDS modulator
    set_rds_pi(data->pi); //[cite: 5]
    set_rds_ecc(data->ecc); //[cite: 5]
    set_rds_lic(data->lic); //[cite: 5]
    set_rds_ps(data->ps); //[cite: 5]
    if (data->lps) set_rds_lps(data->lps);
    set_rds_rt(data->rt); //[cite: 5]
    set_rds_ptyn(data->ptyn); //[cite: 5]
    set_rds_pty(data->pty); //[cite: 5]
    set_rds_ab(0); //[cite: 5]
    set_rds_ms(1); //[cite: 5]
    set_rds_tp(data->tp); //[cite: 5]
    set_rds_ta(data->ta); //[cite: 5]
    if(dstereo == 1) { //[cite: 5]
        set_rds_di(0); //[cite: 5]
    } else {
        set_rds_di(1); //[cite: 5]
    }
    if(data->log) { //[cite: 5]
        if(drds == 1) { //[cite: 5]
            printf("RDS Disabled (you can enable with control fifo with the RDS command)\n"); //[cite: 5]
        } else {
            printf("PI: %04X, ECC: %02X, LIC: %02X, PS: \"%s\".\n", data->pi, data->ecc, data->lic, data->ps); //[cite: 5]
            if (data->lps) printf("LPS: \"%s\"\n", data->lps);
            printf("RT: \"%s\"\n", data->rt); //[cite: 5]
            printf("PTYN: \"%s\"\n", data->ptyn); //[cite: 5]

            if(data->af_array[0]) { //[cite: 5]
                set_rds_af(data->af_array); //[cite: 5]
                printf("AF: "); //[cite: 5]
                int f; //[cite: 5]
                for(f = 1; f < data->af_array[0]+1; f++) { //[cite: 5]
                    printf("%f Mhz ", (float)(data->af_array[f]+875)/10); //[cite: 5]
                }
                printf("\n"); //[cite: 5]
            }
        }
    }

    // Initialize the control pipe reader
    if(data->control_pipe) { //[cite: 5]
        if(open_control_pipe(data->control_pipe) == 0) { //[cite: 5]
            if(data->log) printf("Reading control commands on %s.\n", data->control_pipe); //[cite: 5]
        } else {
            if(data->log) printf("Failed to open control pipe: %s.\n", data->control_pipe); //[cite: 5]
            data->control_pipe = NULL; //[cite: 5]
        }
    }
    if(data->log) printf("Starting to transmit on %3.1f MHz.\n", data->carrier_freq/1e6); //[cite: 5]
    float deviation_scale_factor; //[cite: 5]
    deviation_scale_factor = 0.1 * (data->deviation); //[cite: 5]
    int paused = 0; //[cite: 5]
    for (;;)
    {
        if(data->control_pipe) { //[cite: 5]
            ResultAndArg pollResult = poll_control_pipe(data->log); //[cite: 5]
            if(pollResult.res == CONTROL_PIPE_RDS_SET) { //[cite: 5]
                drds = pollResult.arg_int; //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_PWR_SET) { //[cite: 5]
                padgpio gpiopad; //[cite: 5]
                gpiopad.setlevel(pollResult.arg_int); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_DEVIATION_SET) { //[cite: 5]
                data->deviation = std::stoi(pollResult.arg); //[cite: 5]
                deviation_scale_factor = 0.1 * (data->deviation); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_STEREO_SET) { //[cite: 5]
                dstereo = pollResult.arg_int; //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_GAIN_SET) { //[cite: 5]
                audio_gain = std::stof(pollResult.arg); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSORDECAY_SET) { //[cite: 5]
                compressor_decay = std::stof(pollResult.arg); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSORATTACK_SET) { //[cite: 5]
                compressor_attack = std::stof(pollResult.arg); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_CT_SET) { //[cite: 5]
                rds_ct_enabled = pollResult.arg_int; //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_RDSVOL_SET) { //[cite: 5]
                rds_volume = std::stof(pollResult.arg); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_PAUSE_SET) { //[cite: 5]
                paused = pollResult.arg_int; //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_MPXGEN_SET) { //[cite: 5]
                generate_multiplex = pollResult.arg_int; //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSOR_SET) { //[cite: 5]
                enablecompressor = pollResult.arg_int; //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSORMAXGAINRECIP_SET) { //[cite: 5]
                compressor_max_gain_recip = std::stof(pollResult.arg); //[cite: 5]
            } else if(pollResult.res == CONTROL_PIPE_LIMITERTHRESHOLD_SET) { //[cite: 5]
                limiter_threshold = std::stof(pollResult.arg); //[cite: 5]
            }
        }
        fm_mpx_data *data2; //[cite: 5]
        data2 = (fm_mpx_data *)malloc(sizeof(fm_mpx_data)); //[cite: 5]
        data2->drds = drds; //[cite: 5]
        data2->compressor_decay = compressor_decay; //[cite: 5]
        data2->compressor_attack = compressor_attack; //[cite: 5]
        data2->compressor_max_gain_recip = compressor_max_gain_recip; //[cite: 5]
        data2->dstereo = dstereo; //[cite: 5]
        data2->audio_gain = audio_gain; //[cite: 5]
        data2->enablecompressor = enablecompressor; //[cite: 5]
        data2->rds_ct_enabled = rds_ct_enabled; //[cite: 5]
        data2->rds_volume = rds_volume; //[cite: 5]
        data2->paused = paused; //[cite: 5]
        data2->generate_multiplex = generate_multiplex; //[cite: 5]
        data2->limiter_threshold = limiter_threshold; //[cite: 5]
        if(fm_mpx_get_samples(audio_data, data2) < 0 ) terminate(0); //[cite: 5]
        data_len = DATA_SIZE; //[cite: 5]
        for(int i=0;i< data_len;i++) { //[cite: 5]
            devfreq[i] = audio_data[i]*deviation_scale_factor; //[cite: 5]
        }
        fmmod->SetFrequencySamples(devfreq,data_len); //[cite: 5]
    }
    return 0; //[cite: 5]
}

int main(int argc, char **argv) { //[cite: 5]
    tx_data data = {
        .carrier_freq = 100000000, //[cite: 5]
        .audio_file = NULL, //[cite: 5]
        .pi = 0x00ff, //[cite: 5]
        .ecc = 0x0, //[cite: 5]
        .lic = 0x0, //[cite: 5]
        .ps = "Pi-FmSa", //[cite: 5]
        .lps = NULL,
        .rt = "Broadcasting on a Raspberry Pi: Simply Advanced", //[cite: 5]
        .ptyn = "OTHER", //[cite: 5]
        .control_pipe = NULL, //[cite: 5]
        .pty = 0, //[cite: 5]
        .af_array = {0}, //[cite: 5]
        .raw = 0, //[cite: 5]
        .drds = 0, //[cite: 5]
        .preemp = 50e-6, //[cite: 5]
        .power = 7, //[cite: 5]
        .rawSampleRate = 44100, //[cite: 5]
        .rawChannels = 2, //[cite: 5]
        .deviation = 75000, //[cite: 5]
        .ta = 0, //[cite: 5]
        .tp = 0, //[cite: 5]
        .cutoff_freq = 15000, //[cite: 5]
        .audio_gain = 1, //[cite: 5]
        .compressor_decay = 0.999995, //[cite: 5]
        .compressor_attack = 1.0, //[cite: 5]
        .compressor_max_gain_recip = 0.01, //[cite: 5]
        .enablecompressor = 1, //[cite: 5]
        .rds_ct_enabled = 1, //[cite: 5]
        .rds_volume = 1.0, //[cite: 5]
        .disablestereo = 0, //[cite: 5]
        .log = 1, //[cite: 5]
        .limiter_threshold = 0.9, //[cite: 5]
    };

    int af_size = 0; //[cite: 5]
    int alternative_freq[100] = {}; //[cite: 5]

    for(int i=1; i<argc; i++) { //[cite: 5]
        char *arg = argv[i]; //[cite: 5]
        char *param = NULL; //[cite: 5]
        if(arg[0] == '-' && i+1 < argc) param = argv[i+1]; //[cite: 5]
        if((strcmp("-audio", arg)==0) && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.audio_file = param; //[cite: 5]
        } else if(strcmp("-freq", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.carrier_freq = (uint32_t)(atof(param)*1e6); //[cite: 5]
            if(data.carrier_freq < 64e6 || data.carrier_freq > 108e6) fatal("Incorrect frequency specification. Must be in megahertz, of the form 107.9, between 64 and 108.\n"); //[cite: 5]
        } else if(strcmp("-pi", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.pi = (uint16_t)strtoul(param, NULL, 16); //[cite: 5]
        } else if(strcmp("-ecc", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.ecc = (uint16_t)strtoul(param, NULL, 16); //[cite: 5]
        } else if(strcmp("-lic", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.lic = (uint16_t)strtoul(param, NULL, 16); //[cite: 5]
        } else if(strcmp("-ps", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.ps = param; //[cite: 5]
        } else if(strcmp("-lps", arg)==0 && param != NULL) {
            i++;
            data.lps = param;
        } else if(strcmp("-rt", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.rt = param; //[cite: 5]
        } else if(strcmp("-ptyn", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.ptyn = param; //[cite: 5]
        } else if(strcmp("-compressordecay", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.compressor_decay = atof(param); //[cite: 5]
        } else if(strcmp("-compressorattack", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.compressor_attack = atof(param); //[cite: 5]
        } else if(strcmp("-compressormaxgainrecip", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.compressor_max_gain_recip = atof(param); //[cite: 5]
        } else if(strcmp("-limiterthreshold", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.limiter_threshold = atof(param); //[cite: 5]
            if(data.limiter_threshold > 3.5) { //[cite: 5]
                fatal("Limiter threshold too high!\n"); //[cite: 5]
            }
        } else if(strcmp("-rdsvolume", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.rds_volume = atof(param); //[cite: 5]
        } else if(strcmp("-pty", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.pty = atoi(param); //[cite: 5]
        } else if(strcmp("-disablelogging", arg)==0) { //[cite: 5]
            data.log = 0; //[cite: 5]
        } else if(strcmp("-ta", arg)==0) { //[cite: 5]
            data.ta = 1; //[cite: 5]
        } else if(strcmp("-tp", arg)==0) { //[cite: 5]
            data.tp = 1; //[cite: 5]
        } else if(strcmp("-ctl", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.control_pipe = param; //[cite: 5]
        } else if(strcmp("-deviation", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            if(strcmp("mono", param)==0) { //[cite: 5]
                data.deviation = 50000; //[cite: 5]
            } else if(strcmp("nfm", param)==0) { //[cite: 5]
                data.deviation = 2500; //[cite: 5]
            }
            else {
                data.deviation = atoi(param); //[cite: 5]
            }
        } else if(strcmp("-rawchannels", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.rawChannels = atoi(param); //[cite: 5]
        } else if(strcmp("-rawsamplerate", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.rawSampleRate = atoi(param); //[cite: 5]
        } else if(strcmp("-cutofffreq", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.cutoff_freq = atof(param); //[cite: 5]
        } else if(strcmp("-audiogain", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            data.audio_gain = atof(param); //[cite: 5]
        } else if(strcmp("-power", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            int tpower = atoi(param); //[cite: 5]
            if(tpower > 7 || tpower < 0) fatal("Power can be between 0 and 7"); //[cite: 5]
            data.power = tpower; //[cite: 5]
        } else if(strcmp("-raw", arg)==0) { //[cite: 5]
            data.raw = 1; //[cite: 5]
        } else if(strcmp("-disablerds", arg)==0) { //[cite: 5]
            data.drds = 1; //[cite: 5]
        } else if(strcmp("-disablestereo", arg)==0) { //[cite: 5]
            data.disablestereo = 1; //[cite: 5]
        } else if(strcmp("-disablecompressor", arg)==0) { //[cite: 5]
            data.enablecompressor = 0; //[cite: 5]
        } else if(strcmp("-disablect", arg)==0) { //[cite: 5]
            data.rds_ct_enabled = 0; //[cite: 5]
        } else if(strcmp("-preemphasis", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            if(strcmp("us", param)==0) { //[cite: 5]
                data.preemp = 75e-6; //[cite: 5]
            } else if(strcmp("eu", param)==0) { //[cite: 5]
                data.preemp = 50e-6; //[cite: 5]
            } else if(strcmp("off", param)==0 || strcmp("0", param)==0) { //[cite: 5]
                data.preemp = 0; //[cite: 5]
            } else {
                data.preemp = atof(param) * 1e-6; //[cite: 5]
            }
        } else if(strcmp("-af", arg)==0 && param != NULL) { //[cite: 5]
            i++; //[cite: 5]
            af_size++; //[cite: 5]
            alternative_freq[af_size] = (int)(10*atof(param))-875; //[cite: 5]
            if(alternative_freq[af_size] < 1 || alternative_freq[af_size] > 204) //[cite: 5]
                fatal("Alternative Frequency has to be set in range of 87.6 Mhz - 107.9 Mhz\n"); //[cite: 5]
        }
        else {
            fatal("Unrecognised argument: %s.\n"
            "Syntax: pi_fm_rds [-freq freq] [-audio file] [-pi pi_code] [-ecc ecc_code] [-lic lic_code]\n"
            "                  [-ps ps_text] [-lps long_ps_text] [-rt rt_text] [-ptyn ptyn_text] [-ctl control_pipe] [-pty program_type]\n", arg); //[cite: 5]
        }
    }

    alternative_freq[0] = af_size; //[cite: 5]
    memcpy(data.af_array, alternative_freq, sizeof(alternative_freq)); //[cite: 5]
    int FifoSize = DATA_SIZE * 2; //[cite: 5]
    fmmod = new ngfmdmasync(data.carrier_freq, 228000, 14, FifoSize, false); //[cite: 5]
    int errcode = tx(&data); //[cite: 5]
    terminate(errcode); //[cite: 5]
}#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>
extern "C"
{
#include "rds.h"
#include "fm_mpx.h"
#include "control_pipe.h"
}
#include <librpitx/librpitx.h>
ngfmdmasync *fmmod;
#define DATA_SIZE 5000
static void terminate(int num)
{
    delete fmmod;
    fm_mpx_close();
    close_control_pipe();
    exit(num);
}

static void fatal(char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

typedef struct tx_data {
    uint32_t carrier_freq;
    char *audio_file;
    uint16_t pi;
    uint16_t ecc;
    uint16_t lic; // Added LIC
    char *ps;
    char *rt;
    char *ptyn;
    char *control_pipe;
    uint8_t pty;
    int af_array[100];
    uint8_t raw;
    uint8_t drds;
    float preemp;
    int power;
    int rawSampleRate;
    int rawChannels;
    int deviation;
    uint8_t ta;
    uint8_t tp;
    float cutoff_freq;
    float audio_gain;
    float compressor_decay;
    float compressor_attack;
    float compressor_max_gain_recip;
    uint8_t enablecompressor;
    uint8_t rds_ct_enabled;
    float rds_volume;
    uint8_t disablestereo;
    uint8_t log;
    float limiter_threshold;
} tx_data;

int tx(tx_data *data) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = terminate;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGKILL, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL); //https://www.gnu.org/software/libc/manual/html_node/Termination-Signals.html
    sigaction(SIGPWR, &sa, NULL);
    sigaction(SIGTSTP, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL); //seg fault
    
    // Data structures for baseband data
    float audio_data[DATA_SIZE];
    float devfreq[DATA_SIZE];
    int data_len = 0;
    int data_index = 0;

    int generate_multiplex = 1;
    int dstereo = data->disablestereo;
    int drds = data->drds;
    float audio_gain = data->audio_gain;
    float compressor_decay = data->compressor_decay;
    float compressor_attack = data->compressor_attack;
    float compressor_max_gain_recip = data->compressor_max_gain_recip;
    int enablecompressor = data->enablecompressor;
    int rds_ct_enabled = data->rds_ct_enabled;
    float rds_volume = data->rds_volume;
    float limiter_threshold = data->limiter_threshold;

    //set the power
    padgpio gpiopad;
    gpiopad.setlevel(data->power);

    // Initialize the baseband generator
    if(fm_mpx_open(data->audio_file, DATA_SIZE, data->raw, data->preemp, data->rawSampleRate, data->rawChannels, data->cutoff_freq) < 0) return 1;

    // Initialize the RDS modulator
    char myps[9] = {0};
    set_rds_pi(data->pi);
    set_rds_ecc(data->ecc);
    set_rds_lic(data->lic); // Added LIC initialization
    set_rds_ps(data->ps);
    set_rds_rt(data->rt);
    set_rds_ptyn(data->ptyn);
    set_rds_pty(data->pty);
    set_rds_ab(0);
    set_rds_ms(1); // yes
    set_rds_tp(data->tp);
    set_rds_ta(data->ta);
    if(dstereo == 1) {
        set_rds_di(0);
    } else {
        set_rds_di(1);
    }
    uint16_t count = 0;
    uint16_t count2 = 0;
    if(data->log) {
        if(drds == 1) {
            printf("RDS Disabled (you can enable with control fifo with the RDS command)\n");
        } else {
            printf("PI: %04X, ECC: %02X, LIC: %02X, PS: \"%s\".\n", data->pi, data->ecc, data->lic, data->ps);
            printf("RT: \"%s\"\n", data->rt);
            printf("PTYN: \"%s\"\n", data->ptyn);

            if(data->af_array[0]) {
                set_rds_af(data->af_array);
                printf("AF: ");
                int f;
                for(f = 1; f < data->af_array[0]+1; f++) {
                    printf("%f Mhz ", (float)(data->af_array[f]+875)/10);
                }
                printf("\n");
            }
        }
    }

    // Initialize the control pipe reader
    if(data->control_pipe) {
        if(open_control_pipe(data->control_pipe) == 0) {
            if(data->log) printf("Reading control commands on %s.\n", data->control_pipe);
        } else {
            if(data->log) printf("Failed to open control pipe: %s.\n", data->control_pipe);
            data->control_pipe = NULL;
        }
    }
    if(data->log) printf("Starting to transmit on %3.1f MHz.\n", data->carrier_freq/1e6);
    float deviation_scale_factor;
    // The deviation specifies how wide the signal is (from its lowest bandwidht to its highest, but not including sub-carriers). 
    // Use 75kHz for WFM (broadcast radio, or 50khz can be used)
    // and about 2.5kHz for NFM (walkie-talkie style radio)
    deviation_scale_factor=  0.1 * (data->deviation);
    int paused = 0;
    for (;;)
    {
        if(data->control_pipe) {
            ResultAndArg pollResult = poll_control_pipe(data->log);
            if(pollResult.res == CONTROL_PIPE_RDS_SET) {
                drds = pollResult.arg_int;
            } else if(pollResult.res == CONTROL_PIPE_PWR_SET) {
                padgpio gpiopad;
                gpiopad.setlevel(pollResult.arg_int);
            } else if(pollResult.res == CONTROL_PIPE_DEVIATION_SET) {
                data->deviation = std::stoi(pollResult.arg);
                deviation_scale_factor=  0.1 * (data->deviation);
            } else if(pollResult.res == CONTROL_PIPE_STEREO_SET) {
                dstereo = pollResult.arg_int;
            } else if(pollResult.res == CONTROL_PIPE_GAIN_SET) {
                audio_gain = std::stof(pollResult.arg);
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSORDECAY_SET) {
                compressor_decay = std::stof(pollResult.arg);
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSORATTACK_SET) {
                compressor_attack = std::stof(pollResult.arg);
            } else if(pollResult.res == CONTROL_PIPE_CT_SET) {
                rds_ct_enabled = pollResult.arg_int;
            } else if(pollResult.res == CONTROL_PIPE_RDSVOL_SET) {
                rds_volume = std::stof(pollResult.arg);
            } else if(pollResult.res == CONTROL_PIPE_PAUSE_SET) {
                paused = pollResult.arg_int;
            } else if(pollResult.res == CONTROL_PIPE_MPXGEN_SET) {
                generate_multiplex = pollResult.arg_int;
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSOR_SET) {
                enablecompressor = pollResult.arg_int;
            } else if(pollResult.res == CONTROL_PIPE_COMPRESSORMAXGAINRECIP_SET) {
                compressor_max_gain_recip = std::stof(pollResult.arg);
            } else if(pollResult.res == CONTROL_PIPE_LIMITERTHRESHOLD_SET) {
                limiter_threshold = std::stof(pollResult.arg);
            }
        }
        fm_mpx_data *data2;
        data2 = (fm_mpx_data *)malloc(sizeof(fm_mpx_data));
        data2->drds = drds;
        data2->compressor_decay = compressor_decay;
        data2->compressor_attack = compressor_attack;
        data2->compressor_max_gain_recip = compressor_max_gain_recip;
        data2->dstereo = dstereo;
        data2->audio_gain = audio_gain;
        data2->enablecompressor = enablecompressor;
        data2->rds_ct_enabled = rds_ct_enabled;
        data2->rds_volume = rds_volume;
        data2->paused = paused;
        data2->generate_multiplex = generate_multiplex;
        data2->limiter_threshold = limiter_threshold;
        if(fm_mpx_get_samples(audio_data, data2) < 0 ) terminate(0);
        data_len = DATA_SIZE;
        for(int i=0;i< data_len;i++) {
            devfreq[i] = audio_data[i]*deviation_scale_factor;
        }
        fmmod->SetFrequencySamples(devfreq,data_len);
    }
    return 0;
}


int main(int argc, char **argv) {
    tx_data data = {
        .carrier_freq = 100000000,
        .audio_file = NULL,
        .pi = 0x00ff,
        .ecc = 0x0,
        .lic = 0x0, // Added default LIC value
        .ps = "Pi-FmSa",
        .rt = "Broadcasting on a Raspberry Pi: Simply Advanced",
        .ptyn = "OTHER",
        .control_pipe = NULL,
        .pty = 0,
        .af_array = {0},
        .raw = 0,
        .drds = 0,
        .preemp = 50e-6,
        .power = 7,
        .rawSampleRate = 44100,
        .rawChannels = 2,
        .deviation = 75000,
        .ta = 0,
        .tp = 0,
        .cutoff_freq = 15000,
        .audio_gain = 1,
        .compressor_decay = 0.999995,
        .compressor_attack = 1.0,
        .compressor_max_gain_recip = 0.01,
        .enablecompressor = 1,
        .rds_ct_enabled = 1,
        .rds_volume = 1.0,
        .disablestereo = 0,
        .log = 1,
        .limiter_threshold = 0.9,
    };

    int af_size = 0;
    int gpiopin = 4;
    int compressorchanges = 0;
    int limiterchanges = 0;

    int alternative_freq[100] = {};
    int bypassfreqrange = 0;
    // Parse command-line arguments
    for(int i=1; i<argc; i++) {
        char *arg = argv[i];
        char *param = NULL;
        if(arg[0] == '-' && i+1 < argc) param = argv[i+1];
        if((strcmp("-audio", arg)==0) && param != NULL) {
            i++;
            data.audio_file = param;
        } else if(strcmp("-freq", arg)==0 && param != NULL) {
            i++;
            data.carrier_freq = (uint32_t)(atof(param)*1e6);
            if(data.carrier_freq < 64e6 || data.carrier_freq > 108e6) fatal("Incorrect frequency specification. Must be in megahertz, of the form 107.9, between 64 and 108.\n");
        } else if(strcmp("-pi", arg)==0 && param != NULL) {
            i++;
            data.pi = (uint16_t)strtoul(param, NULL, 16);
        } else if(strcmp("-ecc", arg)==0 && param != NULL) {
            i++;
            data.ecc = (uint16_t)strtoul(param, NULL, 16);
        } else if(strcmp("-lic", arg)==0 && param != NULL) { // Added LIC command-line argument parser
            i++;
            data.lic = (uint16_t)strtoul(param, NULL, 16);
        } else if(strcmp("-ps", arg)==0 && param != NULL) {
            i++;
            data.ps = param;
        } else if(strcmp("-rt", arg)==0 && param != NULL) {
            i++;
            data.rt = param;
        } else if(strcmp("-ptyn", arg)==0 && param != NULL) {
            i++;
            data.ptyn = param;
        } else if(strcmp("-compressordecay", arg)==0 && param != NULL) {
            i++;
            data.compressor_decay = atof(param);
            compressorchanges = 1;
        } else if(strcmp("-compressorattack", arg)==0 && param != NULL) {
            i++;
            data.compressor_attack = atof(param);
            compressorchanges = 1;
        } else if(strcmp("-compressormaxgainrecip", arg)==0 && param != NULL) {
            i++;
            data.compressor_max_gain_recip = atof(param);
            compressorchanges = 1;
        } else if(strcmp("-limiterthreshold", arg)==0 && param != NULL) {
            i++;
            data.limiter_threshold = atof(param);
            limiterchanges = 1;
            if(data.limiter_threshold > 3.5) {
                fatal("Limiter threshold too high!\n");
            }
        } else if(strcmp("-rdsvolume", arg)==0 && param != NULL) {
            i++;
            data.rds_volume = atof(param);
        } else if(strcmp("-pty", arg)==0 && param != NULL) {
            i++;
            data.pty = atoi(param);
        } else if(strcmp("-gpiopin", arg)==0 && param != NULL) {
            i++;
            printf("GPIO pin setting disabled, mod librpitx and pifmsa (pifm simply advanced) for this\n");
        } else if(strcmp("-disablelogging", arg)==0) {
            i++;
            data.log = 0;
        } else if(strcmp("-ta", arg)==0) {
            i++;
            data.ta = 1;
        } else if(strcmp("-bfr", arg)==0) {
            i++;
            bypassfreqrange = 1;
        } else if(strcmp("-tp", arg)==0) {
            i++;
            data.tp = 1;
        } else if(strcmp("-ctl", arg)==0 && param != NULL) {
            i++;
            data.control_pipe = param;
        } else if(strcmp("-deviation", arg)==0 && param != NULL) {
            i++;
            if(strcmp("mono", param)==0) {
                data.deviation = 50000;
            } else if(strcmp("nfm", param)==0) {
                data.deviation = 2500;
            }
            else {
                data.deviation = atoi(param);
            }
        } else if(strcmp("-rawchannels", arg)==0 && param != NULL) {
            i++;
            data.rawChannels = atoi(param);
        } else if(strcmp("-rawsamplerate", arg)==0 && param != NULL) {
            i++;
            data.rawSampleRate = atoi(param);
        } else if(strcmp("-cutofffreq", arg)==0 && param != NULL) {
            i++;
            data.cutoff_freq = atof(param);
        } else if(strcmp("-audiogain", arg)==0 && param != NULL) {
            i++;
            data.audio_gain = atof(param);
        } else if(strcmp("-power", arg)==0 && param != NULL) {
            i++;
            int tpower = atoi(param);
            if(tpower > 7 || tpower < 0) fatal("Power can be between 0 and 7");
            data.power = tpower;
        } else if(strcmp("-raw", arg)==0) {
            i++;
            data.raw = 1;
        } else if(strcmp("-disablerds", arg)==0) {
            i++;
            data.drds = 1;
        } else if(strcmp("-disablestereo", arg)==0) {
            i++;
            data.disablestereo = 1;
        } else if(strcmp("-disablecompressor", arg)==0) {
            i++;
            data.enablecompressor = 0;
        } else if(strcmp("-disablect", arg)==0) {
            i++;
            data.rds_ct_enabled = 0;
        } else if(strcmp("-preemphasis", arg)==0 && param != NULL) {
            i++;
            if(strcmp("us", param)==0) {
                data.preemp = 75e-6;
            } else if(strcmp("eu", param)==0) {
                printf("premp eu default but ok\n");
                data.preemp = 50e-6;
            } else if(strcmp("off", param)==0 || strcmp("0", param)==0) {
                data.preemp = 0;
            } else {
                data.preemp = atof(param) * 1e-6;
            }
        } else if(strcmp("-af", arg)==0 && param != NULL) {
            i++;
            af_size++;
            alternative_freq[af_size] = (int)(10*atof(param))-875;
            if(alternative_freq[af_size] < 1 || alternative_freq[af_size] > 204)
                fatal("Alternative Frequency has to be set in range of 87.6 Mhz - 107.9 Mhz\n");
        }
        else {
            fatal("Unrecognised argument: %s.\n"
            "Syntax: pi_fm_rds [-freq freq] [-audio file] [-pi pi_code] [-ecc ecc_code] [-lic lic_code]\n"
            "                  [-ps ps_text] [-rt rt_text] [-ptyn ptyn_text] [-ctl control_pipe] [-pty program_type] [-raw play raw audio from stdin] [-disablerds] [-af alt freq] [-preemphasis us] [-rawchannels when using the raw option you can change this] [-rawsamplerate same business] [-deviation the deviation, default is 75000] [-tp] [-ta]\n", arg);
        }
    }

    alternative_freq[0] = af_size;
    memcpy(data.af_array, alternative_freq, sizeof(alternative_freq));
    int FifoSize=DATA_SIZE*2;
    fmmod=new ngfmdmasync(data.carrier_freq,228000,14,FifoSize, false);
    int errcode = tx(&data);
    terminate(errcode);
}
