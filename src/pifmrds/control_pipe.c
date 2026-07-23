/*
    PiFmAdv - Advanced FM transmitter for the Raspberry Pi
    Copyright (C) 2017 Miegl

    See https://github.com/Miegl/PiFmAdv
*/

#include <stdlib.h> //[cite: 7]
#include <string.h> //[cite: 7]
#include <stdio.h> //[cite: 7]
#include <unistd.h> //[cite: 7]
#include <errno.h> //[cite: 7]

#include "rds.h" //[cite: 7]
#include "control_pipe.h" //[cite: 7]

#define CTL_BUFFER_SIZE 128

int fd; //[cite: 7]
FILE *f_ctl; //[cite: 7]

/*
 * Opens a file (pipe) to be used to control the RDS coder, in non-blocking mode.
 */
int open_control_pipe(char *filename) //[cite: 7]
{
	fd = open(filename, O_RDWR | O_NONBLOCK); //[cite: 7]
	if(fd == -1) return -1; //[cite: 7]

	int flags; //[cite: 7]
	flags = fcntl(fd, F_GETFL, 0); //[cite: 7]
	flags |= O_NONBLOCK; //[cite: 7]
	if( fcntl(fd, F_SETFL, flags) == -1 ) return -1; //[cite: 7]

	f_ctl = fdopen(fd, "r"); //[cite: 7]
	if(f_ctl == NULL) return -1; //[cite: 7]

	return 0; //[cite: 7]
}

/*
 * Polls the control file (pipe), non-blockingly, and if a command is received,
 * processes it and updates the RDS data.
 */
ResultAndArg poll_control_pipe(int log) { //[cite: 7]
	ResultAndArg resarg;
	static char buf[CTL_BUFFER_SIZE]; //[cite: 7]

	char *fifo = fgets(buf, CTL_BUFFER_SIZE, f_ctl); //[cite: 7]

	if(fifo == NULL) { //[cite: 7]
		resarg.res = -1; //[cite: 7]
		return resarg; //[cite: 7]
	}

	if(strlen(fifo) > 3 && fifo[2] == ' ') { //[cite: 7]
		char *arg = fifo+3; //[cite: 7]
		resarg.arg = fifo+3; //[cite: 7]
		if(arg[strlen(arg)-1] == '\n') arg[strlen(arg)-1] = 0; //[cite: 7]
		if(fifo[0] == 'P' && fifo[1] == 'S') { //[cite: 7]
			if (strlen(arg) > 8) arg[8] = 0;
			set_rds_ps(arg); //[cite: 7]
			if(log==1) printf("PS set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res =  CONTROL_PIPE_PS_SET; //[cite: 7]
		}
		else if(fifo[0] == 'R' && fifo[1] == 'T') { //[cite: 7]
			if (strlen(arg) > 64) arg[64] = 0;
			set_rds_ab(0); //[cite: 7]
			set_rds_rt(arg); //[cite: 7]
			if(log==1) printf("RT A set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_RT_SET; //[cite: 7]
		}
        else if(fifo[0] == 'P' && fifo[1] == 'I') { //[cite: 7]
			if (strlen(arg) > 4) arg[4] = 0;
			set_rds_pi((uint16_t) strtol(arg, NULL, 16)); //[cite: 7]
			if(log==1) printf("PI set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_PI_SET; //[cite: 7]
		}
		else if(fifo[0] == 'T' && fifo[1] == 'A') { //[cite: 7]
			int ta = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			set_rds_ta(ta); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set TA to "); //[cite: 7]
				if(ta) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_TA_SET; //[cite: 7]
		}
		else if(fifo[0] == 'T' && fifo[1] == 'P') { //[cite: 7]
			int tp = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			set_rds_tp(tp); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set TP to "); //[cite: 7]
				if(tp) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_TP_SET; //[cite: 7]
		}
		else if(fifo[0] == 'M' && fifo[1] == 'S') { //[cite: 7]
			int ms = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			set_rds_ms(ms); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set MS to "); //[cite: 7]
				if(ms) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_MS_SET; //[cite: 7]
		}
		else if(fifo[0] == 'A' && fifo[1] == 'B') { //[cite: 7]
			int ab = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			set_rds_ab(ab); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set AB to "); //[cite: 7]
				if(ab) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_AB_SET; //[cite: 7]
		} else if(fifo[0] == 'C' && fifo[1] == 'T') { //[cite: 7]
			int ct = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set CT to "); //[cite: 7]
				if(ct) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.arg_int = ct; //[cite: 7]
			resarg.res = CONTROL_PIPE_CT_SET; //[cite: 7]
		} else if(fifo[0] == 'D' && fifo[1] == 'I') { //[cite: 7]
			int di = atoi(arg); //[cite: 7]
			if (di >= 0 && di <= 0xF) { //[cite: 7]
				set_rds_di(di); //[cite: 7]
				if(log==1) { //[cite: 7]
					printf("Set DI to %d\n", di); //[cite: 7]
				}
			}
			else {
				printf("Wrong DI range, range is 0-15\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_GENERIC_SET; //[cite: 7]
		} 
	} else if(strlen(fifo) > 4 && fifo[3] == ' ') { //[cite: 7]
		char *arg = fifo+4; //[cite: 7]
		resarg.arg = fifo+4; //[cite: 7]
		if(arg[strlen(arg)-1] == '\n') arg[strlen(arg)-1] = 0; //[cite: 7]
		if(fifo[0] == 'L' && fifo[1] == 'P' && fifo[2] == 'S') {
			if (strlen(arg) > 32) arg[32] = 0;
			set_rds_lps(arg);
			if(log==1) printf("Long PS set to: \"%s\"\n", arg);
			resarg.res = CONTROL_PIPE_LPS_SET;
		}
		else if(fifo[0] == 'P' && fifo[1] == 'T' && fifo[2] == 'Y') { //[cite: 7]
			int pty = atoi(arg); //[cite: 7]
			if (pty >= 0 && pty <= 31) { //[cite: 7]
				set_rds_pty(pty); //[cite: 7]
				if(log==1) { //[cite: 7]
					if (!pty) { //[cite: 7]
						printf("PTY disabled\n"); //[cite: 7]
					} else {
						printf("PTY set to: %i\n", pty); //[cite: 7]
					}
				}
			}
			else {
				printf("Wrong PTY identifier! The PTY range is 0 - 31.\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_PTY_SET; //[cite: 7]
		} else if(fifo[0] == 'P' && fifo[1] == 'T' && fifo[2] == 'N') { //[cite: 7]
			if (strlen(arg) > 8) arg[8] = 0;
			set_rds_ptyn(arg); //[cite: 7]
			if(log==1) printf("PTYN set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_PTYN_SET; //[cite: 7]
		} else if(fifo[0] == 'E' && fifo[1] == 'C' && fifo[2] == 'C') { //[cite: 7]
			if (strlen(arg) > 2) arg[2] = 0;
			set_rds_ecc((uint16_t) strtol(arg, NULL, 16)); //[cite: 7]
			if(log==1) printf("ECC set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_GENERIC_SET; //[cite: 7]
		} else if(fifo[0] == 'L' && fifo[1] == 'I' && fifo[2] == 'C') { //[cite: 7]
			if (strlen(arg) > 2) arg[2] = 0;
			set_rds_lic((uint16_t) strtol(arg, NULL, 16)); //[cite: 7]
			if(log==1) printf("LIC set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_LIC_SET; //[cite: 7]
		} else if(fifo[0] == 'P' && fifo[1] == 'W' && fifo[2] == 'R') { //[cite: 7]
			int power_level = atoi(arg); //[cite: 7]
			resarg.arg_int = power_level; //[cite: 7]
			if(log==1) printf("POWER set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_PWR_SET; //[cite: 7]
		} else if(fifo[0] == 'R' && fifo[1] == 'T' && fifo[2] == 'B') { //[cite: 7]
			if (strlen(arg) > 64) arg[64] = 0;
			set_rds_ab(1); //[cite: 7]
			set_rds_rt(arg); //[cite: 7]
			if(log==1) printf("RT B set to: \"%s\"\n", arg); //[cite: 7]
			resarg.res = CONTROL_PIPE_RT_SET; //[cite: 7]
		} else if(fifo[0] == 'R' && fifo[1] == 'D' && fifo[2] == 'S') { //[cite: 7]
			int rds = ( strcmp(arg, "OFF") == 0 ); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set RDS to "); //[cite: 7]
				if(rds) printf("OFF\n"); else printf("ON\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_RDS_SET; //[cite: 7]
			resarg.arg_int = rds; //[cite: 7]
		} else if(fifo[0] == 'D' && fifo[1] == 'E' && fifo[2] == 'V') { //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Deviation to "); //[cite: 7]
				printf("%s", arg); //[cite: 7]
				printf("\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_DEVIATION_SET; //[cite: 7]
			resarg.arg = arg; //[cite: 7]
		} else if(fifo[0] == 'G' && fifo[1] == 'A' && fifo[2] == 'I') { //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Gain to "); //[cite: 7]
				printf("%s", arg); //[cite: 7]
				printf("\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_GAIN_SET; //[cite: 7]
			resarg.arg = arg; //[cite: 7]
		} else if(fifo[0] == 'S' && fifo[1] == 'T' && fifo[2] == 'R') { //[cite: 7]
			int togg = ( strcmp(arg, "OFF") == 0 ); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Stereo Toggle to "); //[cite: 7]
				if(!togg) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_STEREO_SET; //[cite: 7]
			resarg.arg_int = togg; //[cite: 7]
		} else if(fifo[0] == 'C' && fifo[1] == 'O' && fifo[2] == 'D') { //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Compressor Decay to "); //[cite: 7]
				printf("%s", arg); //[cite: 7]
				printf("\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_COMPRESSORDECAY_SET; //[cite: 7]
			resarg.arg = arg; //[cite: 7]
		} else if(fifo[0] == 'C' && fifo[1] == 'O' && fifo[2] == 'A') { //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Compressor Attack to "); //[cite: 7]
				printf("%s", arg); //[cite: 7]
				printf("\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_COMPRESSORATTACK_SET; //[cite: 7]
			resarg.arg = arg; //[cite: 7]
		} else if(fifo[0] == 'R' && fifo[1] == 'D' && fifo[2] == 'V') { //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set RDS Volume to "); //[cite: 7]
				printf("%s", arg); //[cite: 7]
				printf("\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_RDSVOL_SET; //[cite: 7]
			resarg.arg = arg; //[cite: 7]
		} else if(fifo[0] == 'P' && fifo[1] == 'A' && fifo[2] == 'U') { //[cite: 7]
			int togg = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set paused to "); //[cite: 7]
				if(togg) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_PAUSE_SET; //[cite: 7]
			resarg.arg_int = togg; //[cite: 7]
		} else if(fifo[0] == 'M' && fifo[1] == 'P' && fifo[2] == 'X') { //[cite: 7]
			int mpx = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Generate MPX to "); //[cite: 7]
				if(mpx) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_MPXGEN_SET; //[cite: 7]
			resarg.arg_int = mpx; //[cite: 7]
		} else if(fifo[0] == 'C' && fifo[1] == 'O' && fifo[2] == 'M') { //[cite: 7]
			int compressor = ( strcmp(arg, "ON") == 0 ); //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Compressor to "); //[cite: 7]
				if(compressor) printf("ON\n"); else printf("OFF\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_COMPRESSOR_SET; //[cite: 7]
			resarg.arg_int = compressor; //[cite: 7]
		} else if(fifo[0] == 'C' && fifo[1] == 'M' && fifo[2] == 'G') { //[cite: 7]
			if(log==1) { //[cite: 7]
				printf("Set Compressor Max Gain Recip to "); //[cite: 7]
				printf("%s", arg); //[cite: 7]
				printf("\n"); //[cite: 7]
			}
			resarg.res = CONTROL_PIPE_COMPRESSORMAXGAINRECIP_SET; //[cite: 7]
			resarg.arg = arg; //[cite: 7]
		} else if(fifo[0] == 'L' && fifo[1] == 'I' && fifo[2] == 'M') { //[cite: 7]
			if(atof(arg) < 4) { //[cite: 7]
				if(log==1) { //[cite: 7]
					printf("Set Limiter Threshold to "); //[cite: 7]
					printf("%s", arg); //[cite: 7]
					printf("\n"); //[cite: 7]
				}
				resarg.res = CONTROL_PIPE_LIMITERTHRESHOLD_SET; //[cite: 7]
				resarg.arg = arg; //[cite: 7]
			} else {
				if(log==1) { //[cite: 7]
					printf("Limiter threshold was not set, thresholds larger than 4 are not allowed\n"); //[cite: 7]
				}
			}
		}
	}
	return resarg; //[cite: 7]
}

int close_control_pipe() { //[cite: 7]
	if(f_ctl) fclose(f_ctl); //[cite: 7]
	if(fd) return close(fd); //[cite: 7]
	else return 0; //[cite: 7]
}
