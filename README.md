# MMC3680KJ

## 简介

本软件包是为MEMSIC(美新半导体)公司的MMC3680KJ三轴磁力计制作的软件包，已经对接好sensor框架，通过sensor框架，开发者可以快速地将此传感器驱动起来。传感器的官方datasheet在lib目录下！

MMC3680KJ仅支持IIC通讯，因此只提供IIC接口。此软件包已经在ali-developer-kit开发板上测试通过。阉割掉了传感器的温度测量功能，因为觉得没必要，如果有朋友需要可以发邮件联系我加上！

## 支持情况
| 包含设备         | 磁力计 |
| --------------- | -------|
| **通讯接口**     |        |
| IIC              | √     |
| SPI              |       |
| **工作模式**     |          |
| 轮询             | √        |
| 中断             |          |
| FIFO             |          |
| **电源模式**     |          |
| 掉电             | √        |
| 低功耗           |          |
| 普通             | √        |
| 高功耗           |          |
| **获取id**      |   √      |
| **数据输出速率** |   √      |
| **测量范围**     |          |
| **自检**         |          |
| **多实例**       |          |

## 使用说明

和其他传感器一样，请查看rtt传感器开发文档。

## 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：mmc3680kj 设备使用 IIC 进行数据通讯，需要系统 IIC 驱动支持；

## 获取软件包

使用 mmc3680kj 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      MMC3680KJ sensor driver package, support: Magnetometer.
              Version (latest)  --->
```

**Version**：软件包版本选择

有人提需求即随缘更新

### 使用软件包

bmp280 软件包初始化函数如下所示：

```
int rt_hw_mmc3680kj_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备）；
- 注册相应的传感器设备，完成 mmc3680kj 设备的注册；

#### 初始化示例

```
#include "sensor_mmc3680kj.h"

int mmc3680_register(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = MMC3680KJ_I2CBUS_NAME;
    cfg.intf.user_data = (void *)MMC3680KJ_ADDR_DEFAULT;

    rt_hw_mmc3680kj_init("mmc3680kj", &cfg);
    return 0;
}
INIT_APP_EXPORT(mmc3680_register);
```
可以先在msh控制台，输入 list_device ，来查看驱动（mag_mmc3）是否挂载成功，成功之后可以使用 sensor_polling mag_mmc3 来查看大气压信息。
## 注意事项

暂无

## 联系人信息

维护人:

- [ZhouRiQiang](https://github.com/mumuge1) 

- 主页：<https://github.com/mumuge1/mmc3680kj>

- 邮箱：<z18279503024@163.com>

- 第一次写驱动，有bug或者问题的欢迎轰炸我