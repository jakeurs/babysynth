#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
// #include "cmd_system.h"
// #include "cmd_wifi.h"
// #include "cmd_nvs.h"
// #define CONFIG_ESP_MAIN_TASK_STACK_SIZE = 7168
#define CONFIG_CONSOLE_STORE_HISTORY = n
#define CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH 255
#define PROMPT_STR "?"

typedef enum SERIALDATA_TARGET
{
    SERIALDATA_TARGET_NONE = 255,
    SERIALDATA_TARGET_ALL = 0,
    SERIALDATA_TARGET_INST0 = 1,
    SERIALDATA_TARGET_INST1 = 2,
    SERIALDATA_TARGET_INST2 = 3,
    SERIALDATA_TARGET_KICK = 4,
    SERIALDATA_TARGET_SNARE = 5,
    SERIALDATA_TARGET_HH = 6
} SERIALDATA_TARGET;

typedef enum SERIALDATA_PARAM
{
    SERIALDATA_PARAM_NONE = 0,
    SERIALDATA_PARAM_VOL_4096,
    SERIALDATA_PARAM_VIEWPARAMS,
    SERIALDATA_PARAM_OB0_V,
    SERIALDATA_PARAM_OB0_FO,
    SERIALDATA_PARAM_OB0_FR,
    SERIALDATA_PARAM_OB0_DT,
    SERIALDATA_PARAM_OB0_A,
    SERIALDATA_PARAM_OB0_D,
    SERIALDATA_PARAM_OB0_S,
    SERIALDATA_PARAM_OB0_R,
    SERIALDATA_PARAM_OB0_WT,
    SERIALDATA_PARAM_OB1_V,
    SERIALDATA_PARAM_OB1_FO,
    SERIALDATA_PARAM_OB1_FR,
    SERIALDATA_PARAM_OB1_DT,
    SERIALDATA_PARAM_OB1_A,
    SERIALDATA_PARAM_OB1_D,
    SERIALDATA_PARAM_OB1_S,
    SERIALDATA_PARAM_OB1_R,
    SERIALDATA_PARAM_OB1_WT,
    SERIALDATA_PARAM_LP_F,
    SERIALDATA_PARAM_LP_Q
} SERIALDATA_PARAM;

typedef int32_t SERIALDATA_VALUE;

typedef struct SerialDataMessage
{
    SERIALDATA_TARGET target;
    SERIALDATA_PARAM param;
    SERIALDATA_VALUE value;
} SerialDataMessage;

static QueueHandle_t serialInterface_msg_queue;

/* Console command history can be stored to and loaded from a file.
 * The easiest way to do this is to use FATFS filesystem on top of
 * wear_levelling library.
 */

static void serialInterface_sendMessage(SERIALDATA_TARGET target, SERIALDATA_PARAM param, SERIALDATA_VALUE value)
{
    SerialDataMessage m;
    m.target = target;
    m.param = param;
    m.value = value;

    // printf("sending message: target [%ld] param [%ld] value [%ld]\n", (int32_t)m.target, (int32_t)m.param, (int32_t)m.value);

    if (serialInterface_msg_queue != 0)
    {
        if (xQueueSend(serialInterface_msg_queue, (void *)&m, 10) != pdTRUE)
        {
            printf("ERROR: Could not put SerialDataMessage in queue.");
        }
    }
    else
    {
        printf("ERROR: failed to make queue.");
    }
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/* 'vol' command */
static int cli_vol(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("usage: vol [instrument: 1 - 6, 0 = master] [level: 0 - 4096]\n");
        return 0;
    }
    // printf("Setting volume to: %s\r\n", argv[1]);
    serialInterface_sendMessage((SERIALDATA_TARGET)atoi(argv[1]), SERIALDATA_PARAM_VOL_4096, (SERIALDATA_VALUE)atoi(argv[2]));
    return 0;
}
static void register_vol(void)
{
    const esp_console_cmd_t cmd = {
        .command = "vol",
        .help = "Set the overall volume, 0 = master volume",
        .hint = "[instrument: 1 - 6, 0 = master] [level: 0 - 4096]",
        .func = &cli_vol,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
/* 'viewparams' command */
static int cli_viewparams(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("usage: viewparams [instrument: 1 - 6]\n");
        return 0;
    }
    serialInterface_sendMessage((SERIALDATA_TARGET)atoi(argv[1]), SERIALDATA_PARAM_VIEWPARAMS, 0);
    return 0;
}
static void register_viewparams(void)
{
    const esp_console_cmd_t cmd = {
        .command = "viewparams",
        .help = "view the parameters of an instrument",
        .hint = "[instrument: 1 - 6]",
        .func = &cli_viewparams,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* 'viewparams' command */
static int cli_set(int argc, char **argv)
{
    if (argc != 4)
    {
        printf("usage: set [instrument: 1 - 6] [param] [value]\n");
        return 0;
    }
    SERIALDATA_PARAM param = SERIALDATA_TARGET_NONE;
    // parse command
    if (argv[2][0] == 'o' && argv[2][1] == 'b')
    { // oscillatorBank
        uint8_t obTarget = 255;
        if (argv[2][2] == '0')
        {
            obTarget = 0;
        }
        else if (argv[2][2] == '1')
        {
            obTarget = 1;
        }
        else
        {
            printf("unknown OscillatorBank, valid targets are ob0 and ob1\n");
            return 0;
        }
        if (argv[2][3] != '.')
        {
            printf("Invalid oscillatorBank command\n");
            return 0;
        }
        if (argv[2][4] == 'v' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_V;
        else if (argv[2][4] == 'v' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_V;
        else if (argv[2][4] == 'f' && argv[2][5] == 'o' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_FO;
        else if (argv[2][4] == 'f' && argv[2][5] == 'o' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_FO;
        else if (argv[2][4] == 'f' && argv[2][5] == 'r' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_FR;
        else if (argv[2][4] == 'f' && argv[2][5] == 'r' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_FR;
        else if (argv[2][4] == 'd' && argv[2][5] == 't' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_DT;
        else if (argv[2][4] == 'd' && argv[2][5] == 't' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_DT;
        else if (argv[2][4] == 'a' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_A;
        else if (argv[2][4] == 'a' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_A;
        else if (argv[2][4] == 'd' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_D;
        else if (argv[2][4] == 'd' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_D;
        else if (argv[2][4] == 's' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_S;
        else if (argv[2][4] == 's' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_S;
        else if (argv[2][4] == 'r' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_R;
        else if (argv[2][4] == 'r' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_R;
        else if (argv[2][4] == 'w' && argv[2][5] == 't' && obTarget == 0)
            param = SERIALDATA_PARAM_OB0_WT;
        else if (argv[2][4] == 'w' && argv[2][5] == 't' && obTarget == 1)
            param = SERIALDATA_PARAM_OB1_WT;    
    }
    else if (argv[2][0] == 'l' && argv[2][1] == 'p')
    {
        if (argv[2][2] == 'f')
            param = SERIALDATA_PARAM_LP_F;
        else if (argv[2][2] == 'q')
            param = SERIALDATA_PARAM_LP_Q;
    }
    serialInterface_sendMessage((SERIALDATA_TARGET)atoi(argv[1]), param, (SERIALDATA_VALUE)atoi(argv[3]));
    return 0;
}
static void register_set(void)
{
    const esp_console_cmd_t cmd = {
        .command = "set",
        .help = "set a parameter of an instrument e.g., set ob0.v 4096",
        .hint = "[instrument: 1 - 6] [param] [value]",
        .func = &cli_set,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

QueueHandle_t *initialize_SerialInterface()
{

    serialInterface_msg_queue = xQueueCreate(16, sizeof(SerialDataMessage));

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    initialize_nvs();

    /* Register commands */
    esp_console_register_help_command();

    register_vol();
    register_viewparams();
    register_set();

    // register_system();
    // register_wifi();
    // register_nvs();
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    return &serialInterface_msg_queue;
}