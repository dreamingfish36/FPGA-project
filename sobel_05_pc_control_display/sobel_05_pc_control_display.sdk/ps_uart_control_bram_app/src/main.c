#include "xparameters.h"
#include "xuartps.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "xtime_l.h"

#define IMG_WIDTH        128U
#define IMG_HEIGHT       72U
#define RGB888_FORMAT    0x18U
#define UART_BAUD_RATE   115200U
#define UART_WAIT_MS     2000U

#define FRAME_SYNC_0     0x55U
#define FRAME_SYNC_1     0xaaU
#define LINE_SYNC_0      0x33U
#define LINE_SYNC_1      0xccU
#define CTRL_SYNC_0      0xa5U
#define CTRL_SYNC_1      0x5aU

#define CTRL_CMD_MODE    0x01U
#define CTRL_CMD_THRESH  0x02U
#define CTRL_CMD_OVERLAY 0x03U
#define CTRL_CMD_SOURCE  0x04U  // 修改点 S05-SOURCE-PS-A：新增图像来源选择命令，0=GUI UART image，1=FPGA 1080P camera

// ============================================================
// 修改点 S05-10MODE-PS-A：PS 端显示模式同步为 10 个
// 这里的编号必须和：
// 1) PL 端 hdmi_bram_sobel_display.v 里的 MODE_xxx 编号
// 2) PC 端 camera_uart_sender.py 里的 DISPLAY_MODES 映射
// 保持完全一致。
// 控制帧最终写入的是：a5 5a 01 <mode_value>
// ============================================================
#define DISPLAY_MODE_MAX       9U
#define MODE_ORIGINAL          0U   // 原图
#define MODE_GRAY              1U   // 灰度处理
#define MODE_GAUSS             2U   // gauss 高斯滤波
#define MODE_MIDFILTER         3U   // midfilter 中值滤波
#define MODE_SOBEL             4U   // sobel 边缘
#define MODE_RED_OVERLAY       5U   // 红色边缘叠加
#define MODE_BIN               6U   // bin 二值化
#define MODE_RED_ONLY          7U   // 单色输出-R
#define MODE_GREEN_ONLY        8U   // 单色输出-G
#define MODE_BLUE_ONLY         9U   // 单色输出-B
#define SOURCE_GUI_UART        0U   // 修改点 S05-SOURCE-PS-B：使用上位机 UART 发送的 128x72 图像
#define SOURCE_FPGA_CAMERA     1U   // 修改点 S05-SOURCE-PS-C：使用 FPGA 端 1080P 摄像头视频流
#define SOURCE_SELECT_MAX      1U

#ifndef UART_DEVICE_ID
#if defined(XPAR_PS7_UART_1_DEVICE_ID)
#define UART_DEVICE_ID XPAR_PS7_UART_1_DEVICE_ID
#elif defined(XPAR_XUARTPS_1_DEVICE_ID)
#define UART_DEVICE_ID XPAR_XUARTPS_1_DEVICE_ID
#elif defined(XPAR_PS7_UART_0_DEVICE_ID)
#define UART_DEVICE_ID XPAR_PS7_UART_0_DEVICE_ID
#elif defined(XPAR_XUARTPS_0_DEVICE_ID)
#define UART_DEVICE_ID XPAR_XUARTPS_0_DEVICE_ID
#else
#error "No XUartPs device id macro found in xparameters.h"
#endif
#endif

#ifndef FRAMEBUFFER_BASEADDR
#if defined(XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR)
#define FRAMEBUFFER_BASEADDR XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#else
#define FRAMEBUFFER_BASEADDR 0x40000000U
#endif
#endif

#define CTRL_MODE_ADDR      (FRAMEBUFFER_BASEADDR + 0x9000U)
#define CTRL_THRESHOLD_ADDR (FRAMEBUFFER_BASEADDR + 0x9004U)
#define CTRL_OVERLAY_ADDR   (FRAMEBUFFER_BASEADDR + 0x9008U)
#define CTRL_SOURCE_ADDR    (FRAMEBUFFER_BASEADDR + 0x900CU)  // 修改点 S05-SOURCE-PS-D：新增图像来源选择控制字地址

static XUartPs UartInst;
static u8 display_mode = MODE_SOBEL;  // 修改点 S05-10MODE-PS-B：默认显示 Sobel，对应新映射中的 mode=4
static u8 threshold_value = 80U;
static u8 overlay_enable = 0U;
static u8 source_select = SOURCE_GUI_UART;  // 修改点 S05-SOURCE-PS-E：默认保持旧功能，使用 GUI UART image

// ============================================================
// 修改点 S05-10MODE-PS-C：串口回显用的模式名称表
// 这里只用于调试打印，不影响 BRAM 控制字写入。
// ============================================================
static const char *display_mode_names[10] = {
    "original",          // 0 原图
    "gray",              // 1 灰度处理
    "gauss",             // 2 gauss 高斯滤波
    "mid_filter",        // 3 midfilter 中值滤波
    "sobel",             // 4 sobel
    "edge_overlay_red",  // 5 红色边缘叠加
    "binary",            // 6 bin 二值化
    "red",               // 7 单色输出-R
    "green",             // 8 单色输出-G
    "blue"               // 9 单色输出-B
};

// ============================================================
// 修改点 S05-SOURCE-PS-F：串口回显用的图像来源名称表
// source_select=0 表示继续使用 GUI/PC 通过 UART 发送到 BRAM 的图像；
// source_select=1 表示 PL 端选择 FPGA 1080P 摄像头视频流。
// ============================================================
static const char *source_select_names[2] = {
    "gui_uart",     // 0
    "fpga_camera"   // 1
};

static const char *get_source_select_name(u8 source)
{
    if (source <= SOURCE_SELECT_MAX) {
        return source_select_names[source];
    }
    return "invalid";
}

static const char *get_display_mode_name(u8 mode)
{
    if (mode <= DISPLAY_MODE_MAX) {
        return display_mode_names[mode];
    }
    return "invalid";
}

static int uart_recv_byte_timeout(u8 *byte_value, u32 timeout_ms)
{
    XTime start_time;
    XTime now_time;
    XTime timeout_ticks = ((XTime)timeout_ms * (XTime)COUNTS_PER_SECOND) / 1000U;

    if (XUartPs_Recv(&UartInst, byte_value, 1U) == 1U) {
        return XST_SUCCESS;
    }

    XTime_GetTime(&start_time);

    while (1) {
        if (XUartPs_Recv(&UartInst, byte_value, 1U) == 1U) {
            return XST_SUCCESS;
        }

        XTime_GetTime(&now_time);
        if ((now_time - start_time) >= timeout_ticks) {
            return XST_FAILURE;
        }
    }
}

static int uart_recv_u16_le(u16 *value, u32 timeout_ms)
{
    u8 low;
    u8 high;

    if (uart_recv_byte_timeout(&low, timeout_ms) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    if (uart_recv_byte_timeout(&high, timeout_ms) != XST_SUCCESS) {
        return XST_FAILURE;
    }

    *value = (u16)low | ((u16)high << 8);
    return XST_SUCCESS;
}

static void control_write_defaults(void)
{
    Xil_Out32(CTRL_MODE_ADDR, (u32)display_mode);
    Xil_Out32(CTRL_THRESHOLD_ADDR, (u32)threshold_value);
    Xil_Out32(CTRL_OVERLAY_ADDR, (u32)overlay_enable);
    Xil_Out32(CTRL_SOURCE_ADDR, (u32)source_select);
}

static void control_print_state(void)
{
    // 修改点 S05-10MODE-PS-D / S05-SOURCE-PS-G：回显 mode/source 编号和名称，方便确认 GUI 发送是否正确
    xil_printf("control: mode=%d (%s) threshold=%d overlay=%d source=%d (%s)\r\n",
               (u32)display_mode,
               get_display_mode_name(display_mode),
               (u32)threshold_value,
               (u32)overlay_enable,
               (u32)source_select,
               get_source_select_name(source_select));
}

static int handle_control_packet(void)
{
    u8 cmd;
    u8 value;

    if (uart_recv_byte_timeout(&cmd, UART_WAIT_MS) != XST_SUCCESS) {
        return -10;
    }
    if (uart_recv_byte_timeout(&value, UART_WAIT_MS) != XST_SUCCESS) {
        return -11;
    }

    switch (cmd) {
    case CTRL_CMD_MODE:
        // 修改点 S05-10MODE-PS-E：
        // 现在只接受 0~9 这 10 个模式。
        // 对大于 9 的非法模式直接忽略，避免错误命令被写成其他功能。
        if (value <= DISPLAY_MODE_MAX) {
            display_mode = value;
            Xil_Out32(CTRL_MODE_ADDR, (u32)display_mode);
        } else {
            xil_printf("invalid display_mode: %d, ignored\r\n", (u32)value);
            return -13;
        }
        break;

    case CTRL_CMD_THRESH:
        threshold_value = value;
        Xil_Out32(CTRL_THRESHOLD_ADDR, (u32)threshold_value);
        break;

    case CTRL_CMD_OVERLAY:
        overlay_enable = value ? 1U : 0U;
        Xil_Out32(CTRL_OVERLAY_ADDR, (u32)overlay_enable);
        break;

    case CTRL_CMD_SOURCE:
        // 修改点 S05-SOURCE-PS-H：新增图像来源选择命令。
        // value=0：GUI UART image，即保留原来的 PC 发图/PS 写 BRAM/PL 读 BRAM 功能；
        // value=1：FPGA 1080P camera，即 PL 端选择摄像头视频流。
        // 这里不影响 mode/threshold/overlay，也不影响旧的 55 aa 图像帧接收功能。
        if (value <= SOURCE_SELECT_MAX) {
            source_select = value;
            Xil_Out32(CTRL_SOURCE_ADDR, (u32)source_select);
        } else {
            xil_printf("invalid source_select: %d, ignored\r\n", (u32)value);
            return -14;
        }
        break;

    default:
        xil_printf("unknown control command: 0x%02x value=0x%02x\r\n", (u32)cmd, (u32)value);
        return -12;
    }

    control_print_state();
    return 1;
}

static int wait_for_packet_start(u32 timeout_ms)
{
    u8 prev = 0U;
    u8 cur;

    while (1) {
        if (uart_recv_byte_timeout(&cur, timeout_ms) != XST_SUCCESS) {
            return 0;
        }

        if ((prev == FRAME_SYNC_0) && (cur == FRAME_SYNC_1)) {
            return 1;
        }

        if ((prev == CTRL_SYNC_0) && (cur == CTRL_SYNC_1)) {
            return 2;
        }

        prev = cur;
    }
}

static int wait_for_line_sync(u32 timeout_ms)
{
    u8 prev = 0U;
    u8 cur;

    while (1) {
        if (uart_recv_byte_timeout(&cur, timeout_ms) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        if ((prev == LINE_SYNC_0) && (cur == LINE_SYNC_1)) {
            return XST_SUCCESS;
        }
        prev = cur;
    }
}

static void framebuffer_write_pixel(u32 x, u32 y, u8 r, u8 g, u8 b)
{
    u32 offset = ((y * IMG_WIDTH) + x) << 2;
    u32 pixel = ((u32)r << 16) | ((u32)g << 8) | (u32)b;

    Xil_Out32(FRAMEBUFFER_BASEADDR + offset, pixel);
}

static void fill_test_pattern(void)
{
    u32 x;
    u32 y;

    for (y = 0U; y < IMG_HEIGHT; y++) {
        for (x = 0U; x < IMG_WIDTH; x++) {
            u8 r = (u8)((x * 255U) / (IMG_WIDTH - 1U));
            u8 g = (u8)((y * 255U) / (IMG_HEIGHT - 1U));
            u8 b = ((x / 8U) & 1U) ? 0x40U : 0x00U;

            if ((x == 0U) || (x == IMG_WIDTH - 1U) ||
                (y == 0U) || (y == IMG_HEIGHT - 1U)) {
                r = 0xffU;
                g = 0xffU;
                b = 0xffU;
            }

            framebuffer_write_pixel(x, y, r, g, b);
        }
    }
}

static int uart_init(void)
{
    XUartPs_Config *config;
    int status;

    config = XUartPs_LookupConfig(UART_DEVICE_ID);
    if (config == NULL) {
        return XST_FAILURE;
    }

    status = XUartPs_CfgInitialize(&UartInst, config, config->BaseAddress);
    if (status != XST_SUCCESS) {
        return status;
    }

    XUartPs_SetOperMode(&UartInst, XUARTPS_OPER_MODE_NORMAL);
    status = XUartPs_SetBaudRate(&UartInst, UART_BAUD_RATE);
    if (status != XST_SUCCESS) {
        return status;
    }

    return XST_SUCCESS;
}

static int receive_frame_body(void)
{
    u16 width;
    u16 height;
    u8 format;
    u32 row_expected;

    if (uart_recv_u16_le(&width, UART_WAIT_MS) != XST_SUCCESS) {
        return -4;
    }
    if (uart_recv_u16_le(&height, UART_WAIT_MS) != XST_SUCCESS) {
        return -4;
    }
    if (uart_recv_byte_timeout(&format, UART_WAIT_MS) != XST_SUCCESS) {
        return -4;
    }

    if ((width != IMG_WIDTH) || (height != IMG_HEIGHT) || (format != RGB888_FORMAT)) {
        return -1;
    }

    for (row_expected = 0U; row_expected < IMG_HEIGHT; row_expected++) {
        u16 row;
        u32 x;

        if (wait_for_line_sync(UART_WAIT_MS) != XST_SUCCESS) {
            return -5;
        }
        if (uart_recv_u16_le(&row, UART_WAIT_MS) != XST_SUCCESS) {
            return -6;
        }

        if (row != row_expected) {
            return -2;
        }

        for (x = 0U; x < IMG_WIDTH; x++) {
            u8 r;
            u8 g;
            u8 b;

            if (uart_recv_byte_timeout(&r, UART_WAIT_MS) != XST_SUCCESS) {
                return -7;
            }
            if (uart_recv_byte_timeout(&g, UART_WAIT_MS) != XST_SUCCESS) {
                return -7;
            }
            if (uart_recv_byte_timeout(&b, UART_WAIT_MS) != XST_SUCCESS) {
                return -7;
            }
            framebuffer_write_pixel(x, row, r, g, b);
        }
    }

    return 0;
}

int main(void)
{
    int status;
    u32 frame_count = 0U;
    u32 wait_count = 0U;

    status = uart_init();
    if (status != XST_SUCCESS) {
        xil_printf("UART init failed: %d\r\n", status);
        return status;
    }

    xil_printf("\r\nPS UART PL Control HDMI display\r\n");
    xil_printf("BRAM base: 0x%x, baud: %d, image: %dx%d\r\n",
               (u32)FRAMEBUFFER_BASEADDR,
               (u32)UART_BAUD_RATE,
               (u32)IMG_WIDTH,
               (u32)IMG_HEIGHT);
    xil_printf("control frame: a5 5a cmd value, cmd 1=mode 2=threshold 3=overlay 4=source\r\n");
    xil_printf("display mode range: 0~9, default=%d (%s)\r\n",
               (u32)display_mode,
               get_display_mode_name(display_mode));
    xil_printf("source select: 0=gui_uart, 1=fpga_camera, default=%d (%s)\r\n",
               (u32)source_select,
               get_source_select_name(source_select));

    fill_test_pattern();
    control_write_defaults();
    control_print_state();

    while (1) {
        status = wait_for_packet_start(UART_WAIT_MS);
        if (status == 1) {
            status = receive_frame_body();
            if (status == 0) {
                frame_count++;
                wait_count = 0U;
                xil_printf("received frame %d\r\n", frame_count);
            } else {
                xil_printf("frame error: %d\r\n", status);
            }
        } else if (status == 2) {
            (void)handle_control_packet();
            wait_count = 0U;
        } else {
            wait_count++;
            if ((wait_count & 0x3U) == 1U) {
                xil_printf("waiting for frame or control header\r\n");
            }
        }
    }
}
