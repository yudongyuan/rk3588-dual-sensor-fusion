#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "public_cfg.h"
#include "heimann_drv.h"

const char* commands[] = {
    "heimann-snap", "heimann-header", "heimann-hex",
    "color-classic", "color-turbo", "color-hot", "color-viridis",
    "color-inferno", "color-graysr", "color-grays", 
    "yolo-req=0", "yolo-req=1", "edge-req=0", "edge-req=1",
    "exit",
    NULL
};

char* command_generator(const char* text, int state) {
    static int list_index, len;
    const char* name;

    if (!state) {
        list_index = 0;
        len = strlen(text);
    }

    while ((name = commands[list_index++])) {
        if (strncmp(name, text, len) == 0) {
            return strdup(name);  // readline 会 free 这个字符串
        }
    }

    return NULL;
}

char** command_completion(const char* text, int start, int end) {
    // 如果是第一个单词才触发命令补全
    if (start == 0)
        return rl_completion_matches(text, command_generator);
    return NULL;
}

void* cmd_thread(void *arg) {
    thread_context_t* ctx = (thread_context_t*)arg;

    rl_attempted_completion_function = command_completion;

    while (!ctx->cmd_req.exit_req) {
        // char* input = readline(">> ");  // 自动刷新提示符
        char* input = readline("\033[1;33m>> \033[0m");

        if (input == NULL) {
            // 可能是 Ctrl+D 或其他 EOF
            ctx->cmd_req.exit_req = 1;
            break;
        }

        if (strlen(input) > 0) {
            add_history(input);  // 可选：支持方向键历史命令
        }

        if (strncmp(input, "heimann-snap", 12) == 0) {
            ctx->cmd_req.snapshot_request = 1;
        } else if (strncmp(input, "heimann-header", 14) == 0) {
            ctx->cmd_req.print_eeprom_header_req = 1;
        } else if (strncmp(input, "heimann-hex", 11) == 0) {
            ctx->cmd_req.print_eeprom_hex_req = 1;
        } else if (strncmp(input, "exit", 4) == 0) {
            ctx->cmd_req.exit_req = 1;
        } else if (strncmp(input, "color-classic", 13) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_CLASSIC;
        } else if (strncmp(input, "color-turbo", 11) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_TURBO;
        } else if (strncmp(input, "color-hot", 9) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_HOT;
        } else if (strncmp(input, "color-viridis", 13) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_VIRIDIS;
        } else if (strncmp(input, "color-inferno", 13) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_INFERNO;
        } else if (strncmp(input, "color-graysr", 12) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_GRAYSR;
        } else if (strncmp(input, "color-grays", 11) == 0) {
            ctx->cmd_req.colormap_ctrl = COLORMAP_GRAYS;
        } else if (strncmp(input, "yolo-req=0", 10) == 0) {
            ctx->cmd_req.yolo_req = 0;
        } else if (strncmp(input, "yolo-req=1", 10) == 0) {
            ctx->cmd_req.yolo_req = 1;
        } else if (strncmp(input, "edge-req=0", 10) == 0) {
            ctx->cmd_req.edge_req = 0;
        } else if (strncmp(input, "edge-req=1", 10) == 0) {
            ctx->cmd_req.edge_req = 1;
        }
        

        free(input);  // readline 分配了内存，记得释放
    }

    return NULL;
}