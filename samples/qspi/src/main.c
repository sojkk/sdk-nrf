#include <drivers/gpio.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>

#if  defined(CONFIG_NRF52840)
#include <system_nrf53840.h>
#endif
#if defined(CONFIG_NRF5340)
#include <system_nrf5340_application.h>
#endif

#include <hal/nrf_gpio.h>
#include <hal/nrf_qspi.h>
#include <nrfx_qspi.h>

#include <random/rand32.h>


#if  defined(CONFIG_NRF52840)

#define BSP_QSPI_SCK_PIN   19
#define BSP_QSPI_CSN_PIN   17
#define BSP_QSPI_IO0_PIN   20
#define BSP_QSPI_IO1_PIN   21
#define BSP_QSPI_IO2_PIN   22
#define BSP_QSPI_IO3_PIN   23

#endif

#if defined(CONFIG_NRF5340)

#define BSP_QSPI_SCK_PIN   17
#define BSP_QSPI_CSN_PIN   18
#define BSP_QSPI_IO0_PIN   13
#define BSP_QSPI_IO1_PIN   14
#define BSP_QSPI_IO2_PIN   15
#define BSP_QSPI_IO3_PIN   16

#endif

#define QSPI_STD_CMD_WRSR   0x01
#define QSPI_STD_CMD_RSTEN  0x66
#define QSPI_STD_CMD_RST    0x99

#define QSPI_TEST_DATA_SIZE 256

#define WAIT_FOR_PERIPH() do { \
        while (!m_finished) {} \
        m_finished = false;    \
    } while (0)


LOG_MODULE_REGISTER(main, CONFIG_QSPI_APP_LOG_LEVEL);



static volatile bool m_finished = false;
static uint8_t m_buffer_tx[QSPI_TEST_DATA_SIZE];
static uint8_t m_buffer_rx[QSPI_TEST_DATA_SIZE];


static void qspi_handler(nrfx_qspi_evt_t event, void * p_context)
{
    m_finished = true;
}

static void configure_memory()
{
    uint8_t temporary = 0x40;
    int err;
    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode    = QSPI_STD_CMD_RSTEN,
        .length    = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait   = true,
        .wren      = true
    };

    // Send reset enable
    err = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	if(err !=  NRFX_SUCCESS)
	{
	   LOG_ERR("Error on QSPI send reset enable, err = %d", err- NRFX_SUCCESS); 	
	}

    // Send reset command
    cinstr_cfg.opcode = QSPI_STD_CMD_RST;
    err = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	if(err !=  NRFX_SUCCESS)
	{
	   LOG_ERR("Error on QSPI send reset command, err = %d", err- NRFX_SUCCESS); 	
	}

    // Switch to qspi mode
    cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
    err = nrfx_qspi_cinstr_xfer(&cinstr_cfg, &temporary, NULL);
	if(err != NRFX_SUCCESS)
	{
	   LOG_ERR("Error on QSPI switch to QSPI mode, err = %d", err - NRFX_SUCCESS); 	
	}

}



ISR_DIRECT_DECLARE(QSPI_IRQHandler)
{
	nrfx_qspi_irq_handler();
	return 0;
}


void main(void)
{
    uint32_t i;
    int err;


    LOG_INF("QSPI write and read example using 24bit addressing mode");


    for (i = 0; i < QSPI_TEST_DATA_SIZE; ++i)
    {
        m_buffer_tx[i] =  (uint8_t) sys_rand32_get();
    }



    nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG(BSP_QSPI_SCK_PIN , BSP_QSPI_CSN_PIN, BSP_QSPI_IO0_PIN, BSP_QSPI_IO1_PIN, BSP_QSPI_IO2_PIN, BSP_QSPI_IO3_PIN) ;

    err = nrfx_qspi_init(&config, qspi_handler, NULL);
    if(err != NRFX_SUCCESS)
	{
		LOG_ERR("QSPI iniitalization error, err = %d", err - NRFX_SUCCESS);	
		return;
	}
	
    // QSPI IRQ connect
	IRQ_DIRECT_CONNECT(QSPI_IRQn, 6 , QSPI_IRQHandler, 0);

    configure_memory();
	  


    m_finished = false;
    err = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, 0);
	
	LOG_INF("QSPI example started.");
	
	if(err != NRFX_SUCCESS)
	{
		LOG_ERR("QSPI erase error, err = %d", err - NRFX_SUCCESS);	
		return;
	}
   
    WAIT_FOR_PERIPH();
    LOG_INF("Process of erasing first block start");

    err = nrfx_qspi_write(m_buffer_tx, QSPI_TEST_DATA_SIZE, 0);
	if(err != NRFX_SUCCESS)
	{
		LOG_ERR("QSPI write error, err = %d", err);	
		return;
	}
    WAIT_FOR_PERIPH();
    LOG_INF("Process of writing data start");

    err = nrfx_qspi_read(m_buffer_rx, QSPI_TEST_DATA_SIZE, 0);
	if(err != NRFX_SUCCESS)
	{
		LOG_ERR("QSPI read error, err = %d", err - NRFX_SUCCESS);	
		return;
	}
    WAIT_FOR_PERIPH();
    LOG_INF("Data read");

    LOG_INF("Compare...");
    if (memcmp(m_buffer_tx, m_buffer_rx, QSPI_TEST_DATA_SIZE) == 0)
    {
        LOG_INF("Data consistent");
    }
    else
    {
        LOG_INF("Data inconsistent");
    }

    nrfx_qspi_uninit();


}
