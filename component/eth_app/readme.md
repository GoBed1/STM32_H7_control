# Overview
release-v1.0.0-realizes ping and syslog
# Dependence Lib
- ensure ETH(reset network card,set mac address) and LWIP(REF:git@github.com:stm32-hotspot/STM32H7-LwIP-Examples.git) work
# Example
```

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  ip4_addr_t fixed_ip, fixed_netmask, fixed_gateway;
  ip4addr_aton(DEFAULT_IP_ADDR, &fixed_ip);
  ip4addr_aton(DEFAULT_NETMASK, &fixed_netmask);
  ip4addr_aton(DEFAULT_GATEWAY, &fixed_gateway);
  lwip_set_net_param(false, fixed_ip.addr, fixed_netmask.addr, fixed_gateway.addr);

  MX_LWIP_Init();

  osDelay(3000);

  extern struct netif gnetif;
  while (ip4_addr_isany_val(*netif_ip4_addr(&gnetif))) {
    osDelay(500);
  }

  syslog_set_hostname("CraneSensorHub");
  syslog_set_server_ip(DEFAULT_GATEWAY, 514);
  syslog_set_level(LOG_LEVEL_DEBUG);
  syslog_init();

  LOG_INFO("ETH","READY! IP: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
           ip4addr_ntoa(netif_ip4_addr(&gnetif)),
           gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2],
           gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);

  network_ready = true;

  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}

```