# S2-LP
Arduino library to support the S2-LP sub-1GHz transceiver

## API

This sensor uses SPI to communicate.

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    dev_spi = new SPIClass(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi->begin();

An instance can be created and enabled with SPI bus following the procedure below:  

    myS2LP = new S2LP(devSPI, A1, D7, A5, 868000000, 50000000, paInfo, S2LP_GPIO_3, 0x44, 0xEE, 0xFF);
    myS2LP->begin();

In order to be notified when a new packet is received we must register a callback function

    myS2LP->attachS2LPReceive(callback_func);

In order to send a packet to another node with address 0x44 you can use the folowing API:

    myS2LP->send(send_buf, (strlen((char *)send_buf) + 1), 0x44, true);

In order to read a packet after having received a notification you can use the procedure below:

    uint8_t data_size = myS2LP->getRecvPayloadLen();
    myS2LP->read(read_buf, data_size);

## Documentation

You can find the source files at  
https://github.com/stm32duino/S2-LP

The S2-LP sub-1GHz transceiver datasheet is available at  
https://www.st.com/content/st_com/en/products/wireless-transceivers-mcus-and-modules/sub-1ghz-rf/s2-lp.html
