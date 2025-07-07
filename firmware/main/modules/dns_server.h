#ifndef DNS_SERVER_H
#define DNS_SERVER_H

#include "esp_err.h"

// DNS server configuration
typedef struct {
    char *domain_name;      // The domain name to redirect
    char *netif_key;        // The network interface to get the IP from
} dns_server_config_t;

#define DNS_SERVER_CONFIG_SINGLE(DOMAIN_NAME, NETIF_KEY) \
    {                                                    \
        .domain_name = DOMAIN_NAME,                      \
        .netif_key = NETIF_KEY,                          \
    }

/**
 * @brief Starts the DNS server
 *
 * @param config The DNS server configuration
 * @return ESP_OK on success
 */
esp_err_t start_dns_server(const dns_server_config_t *config);

#endif // DNS_SERVER_H
