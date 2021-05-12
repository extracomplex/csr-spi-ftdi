/*
 * spi.c - CSR USB-SPI implementation for FTDI Chip hardware
 * Supported chips:
 *  FT2232C
 *  FT2232H
 *  FT4232H
 *
 *  Author: Filipp Bondarenko (extracomplex@gmail.com)
 *
 *  * This project is a derivative of Frans-Willem Hardijzer's [reverse-engineered
 *    spilpt.dll drivers](https://github.com/Frans-Willem/CsrSpiDrivers);
 *  * Thanks to **unicorn** from <http://www.nebo-forum.kiev.ua/> for the idea of a
 *    DLL for Wine.
 *  * Thanks to all the [Contributors](https://github.com/lorf/csr-spi-ftdi/wiki/Contributors)!
 *
 */

/*
 * FTDI SPI pins in HW mode:
 * PA0/PB0 - SCK
 * PA1/PB1 - MOSI
 * PA2/PB2 - MISO
 * PA3/PB3 - nCS
 *
 * !!! CSR chips require 3V3 or 1V8 I/O level. !!!
 */

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#ifdef SPI_STATS
#include <math.h>
#endif
#include <sys/time.h>

#include <windows.h>
#include <libMPSSE_spi.h>

#include "spi.h"
#include "compat.h"
#include "logging.h"

#ifdef ENABLE_LEDS
# error Sprry, LEDS not implemented in hardware SPI mode
#endif

/* Default SPI clock rate, in kHz */
#define SPIMAXCLOCK     1000

/* SPI device open flag */
static int spi_dev_open = 0;
/* API refs counter */
static int spi_nrefs = 0;

/* FTDI D2XX device handle */
static FT_HANDLE ftdi_hdl;
/* FTDI D2XX device current configuration */
static ChannelConfig ftdi_channelConf;

#ifdef SPI_STATS
static struct spi_stats {
    long reads, writes;
    long read_bytes, write_bytes;
    long ftdi_xfers, ftdi_bytes, ftdi_short_reads;
    struct timeval tv_xfer_begin, tv_xfer;
    struct timeval tv_open_begin, tv_open;
    unsigned long spi_clock_max, spi_clock_min;
    unsigned long slowdowns;
} spi_stats;
#endif

#define SPI_MAX_PORTS   16 /* Max number of FTDI SPI ports */
static struct spi_port spi_ports[SPI_MAX_PORTS];
static int spi_nports = 0; /* SPI ports found in system */

unsigned long spi_clock = 0, spi_max_clock = SPIMAXCLOCK;

/* Error message string buffer */
static char *spi_err_buf = NULL;
/* Error message size */
static size_t spi_err_buf_sz = 0;

void spi_set_err_buf(char *buf, size_t sz)
{
    if (buf && sz) {
        spi_err_buf = buf;
        spi_err_buf_sz = sz;
    } else {
        spi_err_buf = NULL;
        spi_err_buf_sz = 0;
    }
}

/* Trace related macros */
#define SPI_ERR(...)   do { \
        LOG(ERR, __VA_ARGS__); \
        spi_err(__VA_ARGS__); \
    } while (0)

/* SPI_ERR macro implementation */
static void spi_err(const char *fmt, ...) {
    static char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    if (spi_err_buf) {
        strncpy(spi_err_buf, buf, spi_err_buf_sz);
        spi_err_buf[spi_err_buf_sz - 1] = '\0';
    }
    va_end(args);
}

/*
 * FTDI transfer data forth and back in MPSSE  mode
 */

/*
 * spi_xfer_*() use a global read/write buffer ftdi_buf that is flushed on the
 * following conditions:
 *  * when buffer becomes full;
 *  * on the clock change;
 *  * before read operation in spi_xfer();
 *  * if the running status of the CPU is requested from spi_xfer_begin();
 *  * at the closure of FTDI device.
 * Read operations are only done in spi_xfer() and spi_xfer_begin(), in other
 * situations we may safely discard what was read into the buffer by
 * spi_ftdi_xfer().
 */

/* Start SPI transfer */
/* get_status - if set, CSR chip status queried and returned */
int spi_xfer_begin(int get_status)
{
    LOG(DEBUG, ""); /* insert empty line */

    if (spi_clock == 0) {
        SPI_ERR("SPI clock not initialized");
        return -1;
    }

#ifdef SPI_STATS
    if (gettimeofday(&spi_stats.tv_xfer_begin, NULL) < 0)
        LOG(WARN, "gettimeofday failed: %s", strerror(errno));
#endif

    FT_STATUS status = FT_OK;

    /* BlueCore chip SPI port reset sequence: deassert CS, wait at least two
         * clock cycles */
    uint8 out_buf = 0;
    uint32 sizeTransfered;
    status = SPI_ToggleCS(ftdi_hdl, FALSE);
    if(status != FT_OK) {
    	SPI_ERR("FTDI: SPI_ToggleCS() failed status=0x%lx", status);
		return -1;
	}

    /* shift out two BITS for two SPI clock cycles delay */
    status = SPI_Write(ftdi_hdl, &out_buf, 2, &sizeTransfered, SPI_TRANSFER_OPTIONS_SIZE_IN_BITS);
    if(status != FT_OK) {
    	SPI_ERR("FTDI: SPI_Write() failed status=0x%lx", status);
    	return -1;
    }

    /* Start transfer */
    status = SPI_ToggleCS(ftdi_hdl, TRUE);
    if(status != FT_OK) {
		SPI_ERR("FTDI: SPI_ToggleCS() failed status=0x%lx", status);
		return -1;
	}

    if (get_status) {
            /*
             * Read the stopped status of the CPU. From CSR8645 datasheet: "When
             * CSR8645 BGA is deselected (SPI_CS# = 1), the SPI_MISO line does not
             * float. Instead, CSR8645 BGA outputs 0 if the processor is running or
             * 1 if it is stopped". However in practice this is not entirely true.
             * Reading MISO while the CPU is deselected gives wrong result. But
             * reading it just after selecting gives the actual status. Also both
             * sources I consulted (CsrSpiDrivers and CsrUsbSpiDeviceRE) are
             * reading the status after setting CS# to 0.
             */
    	uint8_t dev_status;
    	status = SPI_IsBusy(ftdi_hdl, &dev_status);

    	if(status != FT_OK) {
			SPI_ERR("FTDI: SPI_IsBusy() failed status=0x%lx", status);
			return -1;
		}

		return dev_status ? SPI_CPU_STOPPED : SPI_CPU_RUNNING;
    }
    return 0;
}

/* finish SPI transfer */
int spi_xfer_end(void)
{
    LOG(DEBUG, ""); /* insert empty line */

    FT_STATUS status = FT_OK;

    status = SPI_ToggleCS(ftdi_hdl, FALSE);
	if(status != FT_OK) {
		SPI_ERR("FTDI: SPI_ToggleCS() failed status=0x%lx", status);
		return -1;
	}

#ifdef SPI_STATS
    {
        struct timeval tv;

        if (gettimeofday(&tv, NULL) < 0)
            LOG(WARN, "gettimeofday failed: %s", strerror(errno));
        timersub(&tv, &spi_stats.tv_xfer_begin, &tv);
        timeradd(&spi_stats.tv_xfer, &tv, &spi_stats.tv_xfer);
    }
#endif

    return 0;
}

/* Shift in or out data on SPI bus.
 * cmd - SPI_XFER_READ or SPI_XFER_WRITE command
 * iosize - transfer word size: 8 or 16 bits
 * buf - buffer for transfer data
 * size - transfer size in words
 * return number of words transferred, of -1 if failed
 */
int spi_xfer(int cmd, int iosize, void *buf, int size)
{
    LOG(DEBUG, "(%d, %d, %p, %d)", cmd, iosize, buf, size);

    FT_STATUS status = FT_OK;
    uint32 sizeTransferred;
    uint32 sizeToTransfer;
    uint8 buf16[size * 2]; /* buffer for byte order correction */
    if(iosize == 8) {
    	/* 8 bit mode, nothing to correct */
    	sizeToTransfer = size;
    } else if (iosize == 16) {
    	/* 16 bit mode */
    	sizeToTransfer = size * 2;
    	/* convert from host to SPI byte order */
    	for (int i = 0; i < size; i ++) {
    		uint16_t word = ((uint16_t *)buf)[i];

    		buf16[i*2 + 0] = (uint8)(word >> 8);
    		buf16[i*2 + 1] = (uint8)(word >> 0);
    	}
    } else {
    	SPI_ERR("Unsupported iosize=%u", iosize);
    	return -1;
    }

    if(cmd & SPI_XFER_READ) {
    	if(cmd & SPI_XFER_WRITE) {
    		/* Read&Write operation not used yet, return error */
    		return -1;
    		/*
    		status = SPI_ReadWrite(ftdi_hdl, buf, buf, sizeToTransfer,
    				&sizeTransferred, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
    		*/
    	} else {
    		/* SPI shift data out */
    		status = SPI_Read(ftdi_hdl, buf, sizeToTransfer,
    				&sizeTransferred, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
    	}
    } else if (cmd & SPI_XFER_WRITE) {
    	/* SPI shift data in */
		status = SPI_Write(ftdi_hdl,
				(iosize == 8) ? buf : buf16, sizeToTransfer,
				&sizeTransferred, SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES);
    }
    if(status != FT_OK) {
    	SPI_ERR("FTDI: SPI_Read()/SPI_Write() failed status=0x%lx", status);
    	return -1;
    }

    if((cmd & SPI_XFER_READ) && iosize == 16) {
    	/* convert read data from SPI to host byte order */
		for (int i = 0; i < size; i ++) {
			uint8_t byte_h = ((uint8_t *)buf)[i*2 + 0];
			uint8_t byte_l = ((uint8_t *)buf)[i*2 + 1];
			((uint16_t *)buf)[i] = ((uint16_t)byte_h << 8) | ((uint16_t)byte_l << 0);
		}
    }

#ifdef SPI_STATS
    if (cmd & SPI_XFER_WRITE) {
        spi_stats.writes++;
        spi_stats.write_bytes += size * iosize / 8;
    } else {
        spi_stats.reads++;
        spi_stats.read_bytes += size * iosize / 8;
    }
#endif

    return size;
}

/* Fills spi_ports array with discovered devices, sets spi_nports */
static int spi_enumerate_ports(void)
{
    spi_nports = 0;

    FT_STATUS status = FT_OK;
    uint32 channels = 0;

    status = SPI_GetNumChannels(&channels);
    if(status != FT_OK) {
    	SPI_ERR("FTDI: SPI_GetNumChannels() failed: status=0x%lx", status);
    	return -1;
    }

    if (channels > 0) {
    	/* enumerate discovered SPI ports & fill spi_ports array */
		FT_DEVICE_LIST_INFO_NODE devList = {0};
		for (uint32 i = 0; i < channels; i++) {
			status = SPI_GetChannelInfo(i, &devList);
			if(status != FT_OK) {
				SPI_ERR("FTDI: SPI_GetChannelInfo(%u) failed: status=0x%lx", i, status);
				return -1;
			}

			const char *type_str;

			switch(devList.Type) {
			case FT_DEVICE_2232C:
				type_str = "FT2232C";
				break;

			case FT_DEVICE_2232H:
				type_str = "FT2232H";
				break;

			case FT_DEVICE_4232H:
				type_str = "FT4232H";
				break;

			default:
				type_str = "UNKNOWN";
				break;
			}

			/* copy SPI port meta information */
			sprintf_s(spi_ports[spi_nports].name, sizeof(spi_ports[spi_nports].name), "%s [%s]",
					type_str,
					devList.SerialNumber);
			strcpy_s(spi_ports[spi_nports].manuf, sizeof(spi_ports[spi_nports].manuf), "FTDI");
			strcpy_s(spi_ports[spi_nports].desc, sizeof(spi_ports[spi_nports].desc), devList.Description);
			strcpy_s(spi_ports[spi_nports].serial, sizeof(spi_ports[spi_nports].serial), devList.SerialNumber);
			spi_ports[spi_nports].vid = (uint16_t)((devList.ID >> 16) & 0xffff); /* USB vendor id */
			spi_ports[spi_nports].pid = (uint16_t)((devList.ID >> 0) & 0xffff); /* USB product Id */

			LOG(INFO, "Found device[%u]: name=\"%s\", manuf=\"%s\", desc=\"%s\", serial=\"%s\", vid=0x%04x, pid=0x%04x",
					i,
					spi_ports[spi_nports].name,
					spi_ports[spi_nports].manuf,
					spi_ports[spi_nports].desc,
					spi_ports[spi_nports].serial,
					spi_ports[spi_nports].vid,
					spi_ports[spi_nports].pid);

			spi_nports++;
		}
    }

    return 0;
}

/* initialize FTDI MPSSE lib & discover SPI ports */
int spi_init(void)
{
    LOG(DEBUG, "spi_nrefs=%d, spi_dev_open=%d", spi_nrefs, spi_dev_open);

    spi_nrefs++;

    if (spi_nrefs > 1) {
        LOG(WARN, "Superfluos call to spi_init()");
        return 0;
    }

    LOG(ALL, "csr-spi-ftdi " VERSION ", git rev " GIT_REVISION "\n");

    Init_libMPSSE();

    if (spi_enumerate_ports() < 0) {
        spi_deinit();
        return -1;
    }

    return 0;
}

/* return SPI port list */
int spi_get_port_list(struct spi_port **pportlist, int *pnports)
{
    if (spi_nrefs < 1) {
        SPI_ERR("FTDI: spi not initialized");
        return -1;
    }

    if (pportlist)
        *pportlist = spi_ports;
    if (pnports)
        *pnports = spi_nports;

    return 0;
}

/* reset SPI port list & device open flag */
int spi_deinit(void)
{
    LOG(DEBUG, "spi_nrefs=%d, spi_dev_open=%d", spi_nrefs, spi_dev_open);

    if (spi_nrefs) {
        if (spi_dev_open)
            if (spi_close() < 0)
                return -1;
        spi_nrefs = 0;
    }
    return 0;
}

/* configure actual SPI clock */
/* spi_clk - clock (kHz) */
int spi_set_clock(unsigned long spi_clk) {
    unsigned long ftdi_clk;

    LOG(DEBUG, "(%lu)", spi_clk);


    if (!spi_isopen()) {
        SPI_ERR("FTDI: setting SPI clock failed: SPI device is not open");
        return -1;
    }

    if (spi_clk > spi_max_clock)
        spi_clk = spi_max_clock;

    spi_clock = spi_clk;
    ftdi_clk = spi_clock * 1000; /* clock in Hz */

    LOG(INFO, "FTDI: setting SPI clock to %lu (FTDI baudrate %lu)", spi_clk, ftdi_clk);

    ftdi_channelConf.ClockRate = ftdi_clk;
    FT_STATUS status = FT_OK;

    status = SPI_InitChannel(ftdi_hdl, &ftdi_channelConf);
	if(status != FT_OK) {
		SPI_ERR("FTDI: SPI_InitChannel() failed: status=0x%lx", status);
		return -1;
	}

#ifdef SPI_STATS
    if (spi_stats.spi_clock_max == 0)
        spi_stats.spi_clock_max = spi_max_clock;
    if (spi_stats.spi_clock_min == 0)
        spi_stats.spi_clock_min = spi_max_clock;
    /* Don't account for slow cmds, that are executing at 20 kHz,
     * they are short and not representative */
    if (spi_clock > 20 && spi_clock < spi_stats.spi_clock_min)
            spi_stats.spi_clock_min = spi_clock;
#endif

    return 0;
}

/* set max SPI clock */
/* clk - clock (kHz) */
void spi_set_max_clock(unsigned long clk) {
    LOG(INFO, "FTDI: setting SPI max clock: %lu", clk);
    spi_max_clock = clk;
}

/* slowdown SPI clock */
int spi_clock_slowdown(void) {
    unsigned long clk = spi_clock;

    /* Slow SPI clock down by 1.5 */
    clk = (clk * 2) / 3;
    if (clk < 25)
        clk = 25;

#ifdef SPI_STATS
    spi_stats.slowdowns++;
#endif

    LOG(INFO, "FTDI: SPI clock slowdown, set SPI clock to %lu", clk);
    return spi_set_clock(clk);
}

/* Return max SPI clock setting */
unsigned long spi_get_max_clock(void) {
    return spi_max_clock;
}

/* Return current SPI clock setting */
unsigned long spi_get_clock(void) {
    return spi_clock;
}

/* Open SPI port, switch to MPSSE mode */
int spi_open(int nport)
{
    LOG(DEBUG, "(%d) spi_dev_open=%d", nport, spi_dev_open);

    if (spi_dev_open > 0) {
        LOG(WARN, "Superfluos call to spi_open()");
        return 0;
    }

    LOG(DEBUG, "spi_nports=%d", spi_nports);
    if (spi_nports == 0 || nport > (spi_nports - 1)) {
        SPI_ERR("No FTDI device found");
        goto open_err;
    }
    FT_STATUS status = FT_OK;


#ifdef SPI_STATS
    memset(&spi_stats, 0, sizeof(spi_stats));
    if (gettimeofday(&spi_stats.tv_open_begin, NULL) < 0)
        LOG(WARN, "gettimeofday failed: %s", strerror(errno));
#endif

    status = SPI_OpenChannel(nport, &ftdi_hdl);
    if(status != FT_OK) {
    	SPI_ERR("FTDI: SPI_OpenChannel() failed: status=0x%lx", status);
    	goto open_err;
    }

    /* set device open flag */
    spi_dev_open++;

    LOG(INFO, "FTDI: using FTDI device: \"%s\"", spi_ports[nport].name);

    /* initial FTDI chip configuration for MPSSE mode */
	ftdi_channelConf.ClockRate = I2C_CLOCK_FAST_MODE;
	ftdi_channelConf.LatencyTimer = 1;
	ftdi_channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
	ftdi_channelConf.Pin = (ftdi_channelConf.configOptions & SPI_CONFIG_OPTION_CS_ACTIVELOW) ?
			( ((1<<((ftdi_channelConf.configOptions & SPI_CONFIG_OPTION_CS_MASK)>>2))<<3) << 8 ) : 0;

	status = SPI_InitChannel(ftdi_hdl, &ftdi_channelConf);
	if(status != FT_OK) {
		SPI_ERR("FTDI: SPI_InitChannel() failed: status=0x%lx", status);
		goto open_err;
	}

    return 0;

open_err:
    if (spi_dev_open > 0) {
        SPI_CloseChannel(ftdi_hdl);
    }
    spi_dev_open = 0;

    return -1;
}

/* Return 'device opened' status */
int spi_isopen(void)
{
    return spi_dev_open ? 1 : 0;
}

#ifdef SPI_STATS
void spi_output_stats(void)
{
    double xfer_pct, avg_read, avg_write, rate, iops;
    double ftdi_rate, ftdi_xfers_per_io, avg_ftdi_xfer, ftdi_short_rate;
    struct timeval tv;
    long inxfer_ms;
    FILE *fp;

    fp = log_get_dest();
    if (!fp)
        return;

    /* Calculate timeranges until now */
    if (gettimeofday(&tv, NULL) < 0)
        LOG(WARN, "gettimeofday failed: %s", strerror(errno));
    timersub(&tv, &spi_stats.tv_open_begin, &tv);
    timeradd(&spi_stats.tv_open, &tv, &spi_stats.tv_open);

    xfer_pct = avg_read = avg_write = rate = iops = NAN;
    ftdi_rate = ftdi_xfers_per_io = avg_ftdi_xfer = NAN;

    if (spi_stats.tv_open.tv_sec || spi_stats.tv_open.tv_usec) {
        xfer_pct = (spi_stats.tv_xfer.tv_sec * 1000 + spi_stats.tv_xfer.tv_usec / 1000);
        xfer_pct *= 100;
        xfer_pct /= (spi_stats.tv_open.tv_sec * 1000 + spi_stats.tv_open.tv_usec / 1000);
    }

    if (spi_stats.reads) {
        avg_read = spi_stats.read_bytes;
        avg_read /= spi_stats.reads;
    }

    if (spi_stats.writes) {
        avg_write = spi_stats.write_bytes;
        avg_write /= spi_stats.writes;
    }

    inxfer_ms = spi_stats.tv_xfer.tv_sec * 1000 + spi_stats.tv_xfer.tv_usec / 1000;
    if (inxfer_ms > 0) {
        rate = ((spi_stats.read_bytes + spi_stats.write_bytes) * 1000) /
            inxfer_ms;
        rate /= 1024;   /* In KB/s */

        iops = ((spi_stats.reads + spi_stats.writes) * 1000) / inxfer_ms;

        ftdi_rate = (spi_stats.ftdi_xfers * 1000) / inxfer_ms;

        ftdi_short_rate = (spi_stats.ftdi_short_reads * 1000) / inxfer_ms;
    }

    if (spi_stats.reads || spi_stats.writes)
        ftdi_xfers_per_io = spi_stats.ftdi_xfers / (spi_stats.reads + spi_stats.writes);

    if (spi_stats.ftdi_xfers)
        avg_ftdi_xfer = spi_stats.ftdi_bytes / spi_stats.ftdi_xfers;

    fprintf(fp,
            "*** FTDI Statistics ********************************************************\n"
            "csr-spi-ftdi version: " VERSION " (git rev " GIT_REVISION ")\n"
            "Time open: %ld.%02ld s\n"
            "Time in xfer: %ld.%02ld s (%.2f%% of open time)\n"
            "Reads: %ld (%ld bytes, %.2f bytes avg read size)\n"
            "Writes: %ld (%ld bytes, %.2f bytes avg write size)\n"
            "Xfer data rate: %.2f KB/s (%ld bytes in %ld.%02ld s)\n"
            "IOPS: %.2f IO/s (%ld IOs in %ld.%02ld s)\n"
            "FTDI chip: %s (%d), buffer size: %u bytes\n"
            "FTDI stats: %.2f xfers/s (%.2f short reads/s,\n"
            "            %ld xfers/%ld short reads in %ld.%02ld s,\n"
            "            %.2f xfers/IO, %.2f bytes/xfer)\n"
            "SPI max clock: %lu kHz, min clock: %lu kHz, slowdowns: %lu\n"
            "****************************************************************************\n",
            spi_stats.tv_open.tv_sec, spi_stats.tv_open.tv_usec / 10000,
            spi_stats.tv_xfer.tv_sec, spi_stats.tv_xfer.tv_usec / 10000, xfer_pct,
            spi_stats.reads, spi_stats.read_bytes, avg_read,
            spi_stats.writes, spi_stats.write_bytes, avg_write,
            rate, spi_stats.read_bytes + spi_stats.write_bytes,
                spi_stats.tv_xfer.tv_sec, spi_stats.tv_xfer.tv_usec / 10000,
            iops, spi_stats.reads + spi_stats.writes,
                spi_stats.tv_xfer.tv_sec, spi_stats.tv_xfer.tv_usec / 10000,
            ftdi_type_str, ftdic.type, ftdi_buf_size,
            ftdi_rate, ftdi_short_rate, spi_stats.ftdi_xfers,
                spi_stats.ftdi_short_reads, spi_stats.tv_xfer.tv_sec,
                spi_stats.tv_xfer.tv_usec / 10000, ftdi_xfers_per_io, avg_ftdi_xfer,
            spi_stats.spi_clock_max, spi_stats.spi_clock_min, spi_stats.slowdowns
    );
}
#endif

/* Close SPI port */
int spi_close(void)
{
    LOG(DEBUG, "spi_nrefs=%d, spi_dev_open=%d", spi_nrefs, spi_dev_open);

    if (spi_dev_open) {
        FT_STATUS status = FT_OK;
        status = SPI_CloseChannel(ftdi_hdl);
        if(status != FT_OK) {
        	SPI_ERR("FTDI: SPI_CloseChannel() failed: status=0x%lx", status);
        	return -1;
        }

#ifdef SPI_STATS
        spi_output_stats();
#endif

        /* reset device open flag */
        spi_dev_open = 0;
    }

    return 0;
}
