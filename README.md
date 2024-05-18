# EVB1122

See [EVB1122 | 配备高增益天线的ICL1122 SoC评估开发板](https://www.iclegend.com/zh-hans/product/list/EVB1122/),
which use [ICL1122](https://www.iclegend.com/zh-hans/product/list/ICL1122/) as RF SoC. I believe
[S5KM312CL](https://www.iclegend.com/zh-hans/product/list/S5KM312CL/) is the same chip as ICL1122.

Not sure why this company only provide the datasheet after buying the evaluation board.
Even then, I have no access to the register table/map, only being configured with
the provided SDK. (They provide a tool to generate a register table without telling 
the exact meaning of each register, of course, the tool is not open source)

I choose to not redistribute the configuration tool, until I have a better understanding
to make an open-source version.

- [duilib](https://github.com/duilib/duilib) is used as GUI framework for the configuration tool.

> [!WARNING]
> I have never test the program nor the build artifacts on the real hardware.
> Before I make sure the correctness of the program, please use it with caution
> and problems are expected to be found.

You can find various information about the board in the [docs](docs) directory.
See [the datasheet](docs/DS10012RN_ICL1122_Rev.1.2_20230910.pdf) for more details.

I would post more research results in the future if I have time.

## MCU

MCU is STM32F429VETx, which is a member of STM32F4xx family.

[boards_entry.txt](https://github.com/stm32duino/Arduino_Core_STM32/blob/f31d070d1f2059494c6369ab52808729381f9750/variants/STM32F4xx/F427V(G-I)T_F429V(E-G-I)T_F437V(G-I)T_F439V(G-I)T/boards_entry.txt)

```
GenF4.menu.pnum.GENERIC_F429VETX=Generic F429VETx
GenF4.menu.pnum.GENERIC_F429VETX.upload.maximum_size=524288
GenF4.menu.pnum.GENERIC_F429VETX.upload.maximum_data_size=196608
GenF4.menu.pnum.GENERIC_F429VETX.build.board=GENERIC_F429VETX
GenF4.menu.pnum.GENERIC_F429VETX.build.product_line=STM32F429xx
GenF4.menu.pnum.GENERIC_F429VETX.build.variant=STM32F4xx/F427V(G-I)T_F429V(E-G-I)T_F437V(G-I)T_F439V(G-I)T
```

## Reference

- [FMCWRadar in rev space](https://revspace.nl/FMCWRadar)
- [物联网前沿实践-基于信号传播时间测距](https://iot-book.github.io/12_%E6%97%A0%E7%BA%BF%E6%B5%8B%E8%B7%9D/S2_%E5%9F%BA%E4%BA%8E%E4%BC%A0%E6%92%AD%E6%97%B6%E9%97%B4%E6%B5%8B%E8%B7%9D/)
- [物联网前沿实践-AOA定位算法](https://iot-book.github.io/13_%E6%97%A0%E7%BA%BF%E5%AE%9A%E4%BD%8D/S3_AOA%E5%AE%9A%E4%BD%8D%E7%AE%97%E6%B3%95/)

