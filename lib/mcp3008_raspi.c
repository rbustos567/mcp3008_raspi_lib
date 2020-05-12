/*
 * mcp3008_raspi.c:
 *	Copyright (c) 2020 Ricardo Bustos
 ***********************************************************************
 *
 *    mcp3008 raspberry pi 3+  library is a free software: you can redistribute 
 *    it and/or modify it under the terms of the GNU Lesser General Public 
 *    License as published by the Free Software Foundation, either version 
 *    3 of the License, or (at your option) any later version.
 *
 *    mcp3008 raspberry pi 3+ library is distributed in the hope that it 
 *    will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 *    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *    See the GNU Lesser General Public License for more details.
 *
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "mcp3008_raspi.h"

static struct mcp3008_config *mcp3008_c = NULL;
static struct mcp3008_ioctl_transfer *mcp_tr = NULL;

static int dealloc_mcp3008_config(void);
static int dealloc_mcp3008_io_tr(void);
static int close_mcp3008_dev_path(void);
static int open_mcp3008_dev_path(void); 
static int alloc_mcp3008_io_tr(void);
static int alloc_mcp3008_config(void);
static int set_default_mcp3008_io_tr(void);
static int set_default_mcp3008_conf(void);
static int prepare_mcp3008_spi_msg(unsigned short chan, bool single_mode);
static int convert_mcp3008_rx_buff_to_int(unsigned int *rx_int);
static int convert_mcp3008_int_rx_to_volts(unsigned int input, float *out);

static int (* const init_pfunc[])(void) = {
    alloc_mcp3008_config,
    set_default_mcp3008_conf,
    alloc_mcp3008_io_tr,
    set_default_mcp3008_io_tr,
    open_mcp3008_dev_path
};

static int (* const end_pfunc[])(void) = {
    close_mcp3008_dev_path,
    dealloc_mcp3008_io_tr,
    dealloc_mcp3008_config
};

/*
 * init_mcp3008_device:
 *      Initializes mcp3008 raspberry pi 3+ lib
 *
 *      Required to start the library
 *      It allocates dynamic memory and sets up default 
 *      configurations
 *
 *      Returns zero if no error was found. Otherwise, returns 
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int init_mcp3008_device(void) {

    int ret = MCP3008_NOER;
    uint8_t i;

    uint8_t num_pfuncs = sizeof(init_pfunc) /
                         sizeof(*init_pfunc);

    for (i = 0; i < num_pfuncs && ret == MCP3008_NOER; i++) {
        ret = (*init_pfunc[i])();
    }

    debug_print("",ret);

    return ret;
}

/*
 * end_mcp3008_device:
 *      Closes  mcp3008 raspberry pi 3+ lib
 *
 *      Required to be called after finishing use of lib 
 *      Frees memory and used resources of lib
 *
 *      Returns zero if no error was found. Otherwise, returns 
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int end_mcp3008_device(void) {

    int ret = MCP3008_NOER;
    uint8_t i;

    uint8_t num_pfuncs = sizeof(end_pfunc) /
                         sizeof(*end_pfunc);

    for (i = 0; i < num_pfuncs && ret == MCP3008_NOER; i++) {
        ret = (*end_pfunc[i])();
    }

    debug_print("",ret);

    return ret;
}

/*
 * get_raw_input_from_mcp3008_channel:
 *      Obtains the raw value from the selected analog channel of the mcp3008
 *      args:
 *          chan: selects the analog input channel from the mcp3008 chip
 *                to read from. The value can range between 0 to 7
 *          single_mode: if true it sets in single mode. Otherwise, sets 
 *                       differential mode
 *          output: stores the output raw value received from the mcp3008 chip
 *********************************************************************************
 */
extern int get_raw_input_from_mcp3008_channel(unsigned short chan, bool  single_mode, unsigned int *output) {
    int ret = MCP3008_EANA;

    if ( chan <= MCP3008_MAX_CHN &&
         output != NULL) {

        if (mcp_tr != NULL &&
            mcp_tr->fd > -1) {

            ret = prepare_mcp3008_spi_msg(chan, single_mode);

            if (ret == MCP3008_NOER) {

                if ( ioctl(mcp_tr->fd,
                           SPI_IOC_MESSAGE(1),
                           mcp_tr->tr_msg) > -1) {

                    ret = convert_mcp3008_rx_buff_to_int(output);
                } else {
                    ret = MCP3008_EIOC;
                }
            }
        } else {
            ret = MCP3008_ENUP;
        }
    }

    debug_print("", ret);

    return ret;
}

/*
 * get_volt_input_from_mcp3008_channel:
 *      Obtains the voltage read from the selected analog channel of the mcp3008
 *      args:
 *          chan: selects the analog input channel from the mcp3008 chip
 *                to read from. The value can range between 0 to 7
 *          single_mode: if true it sets in single mode. Otherwise, sets
 *                       differential mode
 *          output: stores in voltage the value received from the mcp3008
 *                  chip. The value can range between 0 and 3.3 v
 *********************************************************************************
 */

extern int get_volt_input_from_mcp3008_channel(unsigned short chan, bool single_mode, float *output) {

    int ret;
    unsigned int raw;

    ret = get_raw_input_from_mcp3008_channel(chan, single_mode, &raw);

    if (ret == MCP3008_NOER) {
        ret = convert_mcp3008_int_rx_to_volts(raw, output);
    }

    debug_print("",ret);

    return ret;
}

/*
 * set_mcp3008_device_file:
 *      Sets the device file for the available spidev driver
 *
 *      args:
 *          dev: path of the spidev driver file
 *
 *      Returns zero if no error was found. Otherwise, returns 
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int set_mcp3008_device_file(const char *dev) {

    int ret = MCP3008_ENUP;

    if (dev != NULL) {
        if ( access(dev, F_OK) == 0) {
            int str_s = sizeof(dev); 

            if (str_s < MCP3008_MAX_PATH) {
                strncpy(mcp3008_c->dev_path, dev, str_s);
                mcp3008_c->dev_path[str_s] = '\0';
		ret = MCP3008_NOER;
            } else {
                ret = MCP3008_ESOV;
            }
    	} else {
            ret = MCP3008_ENAC; 
	}	
    }

    debug_print("",ret);

    return ret;
}

/*
 * set_mcp3008_mode:
 *      Sets the mode for the spidev driver
 *
 *      args:
 *          mode: spi mode
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int set_mcp3008_mode(uint8_t mode) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL &&
        mcp3008_c->spi_msg_c != NULL &&
	mcp_tr != NULL &&
	mcp_tr->fd > -1) {

    	if ( ioctl(mcp_tr->fd, 
		   SPI_IOC_WR_MODE,
		   &mode ) > -1) { 

             mcp3008_c->spi_msg_c->mode = mode;
   	     ret = MCP3008_NOER; 
         } else {
             ret = MCP3008_EIOC; 
         }
    }

    debug_print("", ret);

    return ret;
}

/*
 * set_mcp3008_bits_per_word:
 *      Sets bits per word for the spidev driver
 *
 *      args:
 *          bits: number of bits per word for spi
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int set_mcp3008_bits_per_word(uint8_t bits) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL &&
        mcp3008_c->spi_msg_c != NULL &&
        mcp_tr != NULL ) {

	if (mcp_tr->fd > -1) {    

            if ( ioctl(mcp_tr->fd,
                       SPI_IOC_WR_BITS_PER_WORD,
                       &bits ) > -1) {

                 mcp3008_c->spi_msg_c->bits = bits;
                 ret = MCP3008_NOER;
            } else {
                ret = MCP3008_EIOC;
            }
        } else {
            ret = MCP3008_EFUO; 
        }		
    }

    debug_print("", ret);

    return ret;
}

/*
 * set_mcp3008_speed:
 *      Sets speed communication (Hz) for the spidev driver
 *
 *      args:
 *          speed: hz speed for spi
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int set_mcp3008_speed(uint32_t speed) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL &&
        mcp3008_c->spi_msg_c != NULL &&
        mcp_tr != NULL) {

    	if ( mcp_tr->fd > -1) {
            if ( ioctl(mcp_tr->fd,
                       SPI_IOC_WR_MAX_SPEED_HZ,
                       &speed ) > -1) {

                mcp3008_c->spi_msg_c->speed = speed;
                ret = MCP3008_NOER;
            } else {
	        ret = MCP3008_EIOC;
	    }	    
       	} else {
            ret = MCP3008_EFUO;
        }		
    }

    debug_print("", ret);

    return ret;
}

/*
 * set_mcp3008_delay:
 *      Sets delay time for the spidev driver
 *
 *      args:
 *          delay: delay time
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int set_mcp3008_delay(uint16_t delay) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL &&
        mcp3008_c->spi_msg_c != NULL) {
            
	mcp3008_c->spi_msg_c->delay = delay;
        ret = MCP3008_NOER;
    }

    debug_print("", ret);

    return ret;
}

/*
 * set_mcp3008_cs_change:
 *      Sets or unsets the cs change for the spidev driver
 *
 *      args:
 *          set: true sets the cs change. Otherwise, it unsets it
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
extern int set_mcp3008_cs_change(bool set) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL &&
        mcp3008_c->spi_msg_c != NULL) {
        
	if (set == true)    
	    mcp3008_c->spi_msg_c->cs_change = 1;
	else 
	    mcp3008_c->spi_msg_c->cs_change = 0; 
        
        ret = MCP3008_NOER;
    }

    debug_print("", ret);

    return ret;
}

/*
 * get_mcp3008_error:
 *      Returns description of specified error from mcp3008
 *      raspberry pi 3+ lib
 *
 *      args:
 *          error: error number
 *
 *      Returns a string which describes the specified
 *      error integer
 *********************************************************************************
 */
extern const char *get_mcp3008_error(int error) {

    if (error <= MCP3008_NOER)
        return mcp3008_errors_table[(-error)];

    return NULL;
}

static int dealloc_mcp3008_io_tr(void) {

    int ret = MCP3008_ENUP;

    if (mcp_tr != NULL) {

        if (mcp_tr->tr_msg != NULL) {
            free(mcp_tr->tr_msg);
            mcp_tr->tr_msg = NULL;

            free(mcp_tr);
            mcp_tr = NULL;

            ret = MCP3008_NOER;
        }
    }

    debug_print("", ret);

    return ret;
}

static int close_mcp3008_dev_path(void) {

    int ret = MCP3008_ENUP;

    if (mcp_tr != NULL) {
        ret = close(mcp_tr->fd);
        if (ret == 0) {
            mcp_tr->fd = -1;
            ret = MCP3008_NOER;
	} else {
            ret = MCP3008_EFUC;
        }		
    }

    debug_print("", ret);    

    return ret;
}

static int dealloc_mcp3008_config(void) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL) {
        if (mcp3008_c->dev_path != NULL) {

    	    free(mcp3008_c->dev_path);
	    mcp3008_c->dev_path = NULL;

	    if (mcp3008_c->spi_msg_c != NULL) {

                free(mcp3008_c->spi_msg_c);
	        mcp3008_c->spi_msg_c = NULL;	

                free(mcp3008_c);
                mcp3008_c = NULL;

		ret = MCP3008_NOER;
	    }
        }
    }

    debug_print("", ret);

    return ret;

}	

static int set_default_mcp3008_conf(void) {

    int ret = MCP3008_ENUP;

    if (mcp3008_c != NULL) {
        if (mcp3008_c->dev_path != NULL)	
            strcpy(mcp3008_c->dev_path, MCP3008_DEF_DEV);

        if (mcp3008_c->spi_msg_c != NULL) {
            mcp3008_c->spi_msg_c->mode = MCP3008_DEF_SPI_M;
            mcp3008_c->spi_msg_c->bits = MCP3008_DEF_BITS;
            mcp3008_c->spi_msg_c->speed = MCP3008_DEF_SPEED;
            mcp3008_c->spi_msg_c->delay = MCP3008_DEF_DELAY;
            mcp3008_c->spi_msg_c->cs_change = MCP3008_DEF_CS_CH;
            
	    ret = MCP3008_NOER;
	}
    }

    debug_print("", ret);

    return ret;
}

static int alloc_mcp3008_config(void) {

    int ret = MCP3008_ENNP;

    if (mcp3008_c == NULL) {
        mcp3008_c = (struct mcp3008_config *)malloc 
		    (sizeof(struct mcp3008_config));

        if (mcp3008_c != NULL) {

            mcp3008_c->dev_path = (char *)malloc
                    (MCP3008_MAX_PATH * sizeof(char));
               
	    if (mcp3008_c->dev_path != NULL) {
                
                mcp3008_c->spi_msg_c = (struct spidev_msg_conf *) malloc
                        (sizeof(struct spidev_msg_conf));

	        if ( mcp3008_c->spi_msg_c != NULL) {
                    ret = MCP3008_NOER; 
	        } else {
                    ret = MCP3008_EUAM;
		    goto dealloc_dev_path;
		}
            } else {
		ret = MCP3008_EUAM;
                goto dealloc_mcp3008_c;
            }
        } else {
            ret = MCP3008_EUAM; 
        }
    }

    debug_print("", ret);

    return ret;

dealloc_dev_path:

    free(mcp3008_c->dev_path);
    mcp3008_c->dev_path = NULL;

dealloc_mcp3008_c:
    
    free(mcp3008_c);
    mcp3008_c = NULL;

    debug_print("", ret);

    return ret;
}

static int alloc_mcp3008_io_tr(void) {

    int ret = MCP3008_ENNP;

    if (mcp_tr == NULL) {
        mcp_tr = (struct mcp3008_ioctl_transfer *)malloc
		 (sizeof(struct mcp3008_ioctl_transfer));

	if (mcp_tr != NULL) {
                
	    mcp_tr->tr_msg = (struct spi_ioc_transfer *) malloc
			     (sizeof(struct spi_ioc_transfer)); 

	    if (mcp_tr->tr_msg != NULL) {
		ret = MCP3008_NOER;
	    } else {
		ret = MCP3008_EUAM;
		goto dealloc_mcp_tr;
	    }
        } else {
            ret = MCP3008_EUAM; 
	}	
    }

    debug_print("", ret);

    return ret;

dealloc_mcp_tr:

    free(mcp_tr);
    mcp_tr = NULL;

    debug_print("", ret);

    return ret;
}

static int set_default_mcp3008_io_tr(void) {

    int ret = MCP3008_ENUP;

    if (mcp_tr != NULL) {
        mcp_tr->fd = -1;
    
	if (mcp_tr->tr_msg != NULL) {

    	    mcp_tr->tr_msg->len = MCP3008_SIZE_BUF;
            mcp_tr->tr_msg->delay_usecs = MCP3008_DEF_DELAY;
            mcp_tr->tr_msg->speed_hz = MCP3008_DEF_SPEED;
            mcp_tr->tr_msg->bits_per_word = MCP3008_DEF_BITS;
            mcp_tr->tr_msg->cs_change = MCP3008_DEF_CS_CH;
            mcp_tr->tr_msg->tx_nbits = MCP3008_DEF_TX_NB;
            mcp_tr->tr_msg->rx_nbits = MCP3008_DEF_RX_NB;
            mcp_tr->tr_msg->pad = MCP3008_DEF_PAD;

	    ret = MCP3008_NOER;
	}
    }

    debug_print("",ret);

    return ret;
}

static int open_mcp3008_dev_path(void) {
   
    int ret = MCP3008_ENUP;

    if (mcp_tr != NULL) {
        mcp_tr->fd = open(mcp3008_c->dev_path, O_RDWR );

        if (mcp_tr->fd > -1)
	    ret = MCP3008_NOER;
	else 
	    ret = MCP3008_EFUO; 
    }

    debug_print("",ret);

    return ret;
}

static int prepare_mcp3008_spi_msg(unsigned short chan, bool single_mode) {

    int ret = MCP3008_ENUP;

    if (mcp_tr != NULL) {	
        memset(mcp_tr->s_tx, MCP3008_CLR_HEX, MCP3008_SIZE_BUF);
        memset(mcp_tr->s_rx, MCP3008_CLR_HEX, MCP3008_SIZE_BUF);

	mcp_tr->s_tx[0] = MCP3008_INI_HEX;

        if (single_mode == true)
            mcp_tr->s_tx[1] = MCP3008_S_MODE_HEX;

        mcp_tr->s_tx[1] |= chan << MCP3008_SHIFT_4_B;

        if (mcp_tr->tr_msg != NULL) {

            mcp_tr->tr_msg->tx_buf = (unsigned long)mcp_tr->s_tx;
            mcp_tr->tr_msg->rx_buf = (unsigned long)mcp_tr->s_rx;
            mcp_tr->tr_msg->len = MCP3008_SIZE_BUF;
        
	    if (mcp3008_c != NULL &&
	        mcp3008_c->spi_msg_c != NULL) {
	
	        mcp_tr->tr_msg->delay_usecs = 
                mcp3008_c->spi_msg_c->delay;
        
	        mcp_tr->tr_msg->speed_hz =
                mcp3008_c->spi_msg_c->speed;
        
	        mcp_tr->tr_msg->bits_per_word =
	        mcp3008_c->spi_msg_c->bits;
        
	        mcp_tr->tr_msg->cs_change =
	        mcp3008_c->spi_msg_c->cs_change; 
        
	        mcp_tr->tr_msg->tx_nbits = 
                MCP3008_DEF_TX_NB;

	        mcp_tr->tr_msg->rx_nbits =
	        MCP3008_DEF_RX_NB;
        
	        mcp_tr->tr_msg->pad =
	        MCP3008_DEF_PAD;

                ret = MCP3008_NOER;
	    }
	}
    }

    debug_print("",ret);

    return ret;
}

static int convert_mcp3008_rx_buff_to_int(unsigned int *rx_int) {

    int ret = MCP3008_ENUP;
    unsigned int val = 0;
    unsigned int tmp;
    short i;
    
    if (rx_int != NULL &&
	mcp_tr != NULL &&
	mcp_tr->s_rx != NULL) {

        for (i = MCP3008_SIZE_BUF - 1; i >= 0; i--) {
            if (mcp_tr->s_rx[i] > 0) {
                tmp = mcp_tr->s_rx[i];
                tmp <<= MCP3008_SHIFT_8_B * ( (MCP3008_SIZE_BUF - 1) - i );
                val += tmp; 
            }
        }

	*rx_int = val;

	ret = MCP3008_NOER;
    }

    debug_print("", ret);
    
    return ret;
}

static int convert_mcp3008_int_rx_to_volts(unsigned int input, float *out) {
   
    int ret = MCP3008_ENUP;

    if (out != NULL) {
        *out = ( input * MCP3008_MAX_V_REF ) / (MCP3008_MAX_RAW);
        ret = MCP3008_NOER;
    }

    return ret;
}
