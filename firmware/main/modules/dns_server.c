#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "dns_server.h"
#include "sdkconfig.h"

static const char *TAG = "DNS_SERVER";
#define DNS_PORT 53
#define DNS_MAX_LEN 256

// DNS header structure
#pragma pack(push, 1)
typedef struct {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_header_t;
#pragma pack(pop)

// DNS question structure
#pragma pack(push, 1)
typedef struct {
    uint16_t qtype;
    uint16_t qclass;
} dns_question_t;
#pragma pack(pop)

// DNS answer structure
#pragma pack(push, 1)
typedef struct {
    uint16_t name;
    uint16_t type;
    uint16_t class;
    uint32_t ttl;
    uint16_t rdlength;
    uint32_t rdata;
} dns_answer_t;
#pragma pack(pop)


static esp_netif_t *get_netif_from_key(const char *netif_key)
{
    esp_netif_t *netif = NULL;
    while ((netif = esp_netif_next_unsafe(netif)) != NULL) {
        if (strcmp(esp_netif_get_ifkey(netif), netif_key) == 0) {
            return netif;
        }
    }
    return NULL;
}

static void dns_server_task(void *pvParameters)
{
    dns_server_config_t *config = (dns_server_config_t *)pvParameters;

    esp_netif_t *netif = get_netif_from_key(config->netif_key);
    if (netif == NULL) {
        ESP_LOGE(TAG, "Failed to find netif with key %s", config->netif_key);
        vTaskDelete(NULL);
        return;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);

    uint8_t data[DNS_MAX_LEN];
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(DNS_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS server started on port %d", DNS_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int len = recvfrom(sock, data, DNS_MAX_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (len > 0) {
            dns_header_t *header = (dns_header_t *)data;
            
            // Find the end of the questions
            uint8_t *qname_end = data + sizeof(dns_header_t);
            while (*qname_end != 0) {
                qname_end += (*qname_end + 1);
            }
            
            dns_question_t *question = (dns_question_t *)(qname_end + 1);

            if (ntohs(question->qtype) == 1 && ntohs(question->qclass) == 1) {
                // Prepare the response
                header->flags = htons(0x8180); // Standard response, no error
                header->ancount = htons(1);

                dns_answer_t *answer = (dns_answer_t *)(question + 1);
                answer->name = htons(0xC00C); // Pointer to the name in the question
                answer->type = htons(1); // A record
                answer->class = htons(1); // IN class
                answer->ttl = htonl(300); // 5 minutes
                answer->rdlength = htons(4);
                answer->rdata = ip_info.ip.addr;

                int response_len = (uint8_t *)(answer) + sizeof(dns_answer_t) - data;
                sendto(sock, data, response_len, 0, (struct sockaddr *)&client_addr, client_addr_len);
            }
        }
    }
    close(sock);
    vTaskDelete(NULL);
}

esp_err_t start_dns_server(const dns_server_config_t *config)
{
    xTaskCreate(dns_server_task, "dns_server", 4096, (void *)config, 5, NULL);
    return ESP_OK;
}
