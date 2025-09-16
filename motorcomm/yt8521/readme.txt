MAJOR  = 2
MINOR  = 2
SUBVER = 25077

###########VERSION LIST###########
###########V2.2.25077 Description###########
2023/12/19
1. yt8543 phy initialization updated;
2. yt8531x phy initialization updated to adapt different chip ver;

###########V2.2.24776 Description###########
2023/12/13
1. per 8531x ver register, do different action in config init callback.

###########V2.2.24554 Description###########
2023/12/09
1. fix compile err: "struct phy_device" no "mdio" member in kernel ver 3.10.

###########V2.2.24455 Description###########
2023/12/08
1. enhancement phy driver(omit some redefinition macro and add pause/asymmetric_pause feedback to mac in some PHYs)
2. yt8821 serdes force 2.5G, disable serdes auto negotiation

###########V2.2.24132 Description###########
2023/12/04
1. yt8543 dual port phy driver committed
2. optimization yt8821 phy driver

###########V2.2.23753 Description###########
2023/11/23
1. yt8821 force 2.5G pause/asymmetric_pause capability setting

###########V2.2.23672 Description###########
2023/11/21
1. fixed 8531sc combo mode reg addr space switch fail

###########V2.2.23016 Description###########
2023/11/01
1. yt8521s and yt8531s .read_status callback console output log fixed.

###########V2.2.22933 Description###########
2023/10/30
1. update yt8531x initialization per latest application notes.

###########V2.2.21925 Description###########
2023/09/18
1. update yt8011A config according to latest initialization.

###########V2.2.21636 Description###########
2023/09/11; 
1. update yt8821 mode config in config_init callback

###########V2.2.20110 Description###########
2023/08/04; 
1. updated yt8011 phy driver's config_init callback;
   config ext reg 0x3008 to 0x143
2. updated yt8010as phy driver's config_init callback;
   config ext reg 0x1009 to 0x0
3. updated yt8010a phy driver's config_init callback;
   add yt8010_config_init()

###########V2.2.19820 Description###########
2023/07/28; 
1. add yt8011 automotive phy driver.

###########V2.2.17336 Description###########
2023/06/05; 
1. updated yt8821 init config per latest application notes;

###########V2.2.16841 Description###########
2023/05/19; 
1. add .config_aneg callback in yt8010as driver to support linux kernel 3.18.44
2. add yt8522 phy driver;

###########V2.2.12574 Description###########
2023/02/07; 
1. 合并 YT8512B & YT8512 phy driver. PS: 出货的 YT8512 未明确标识 A/B 版本，且出货的phyid: 0x128
2. add yt8614Q phy driver;
3. .soft_reset callback is added from linux kernel 3.15, so update KERNEL_VERSION(4, 0, 0) to KERNEL_VERSION(3, 15, 0) when using .soft_reset callback
   PS: ver 3.15/.../3.18/3.19/4.3/4.10/4.14/4.17/4.20(4版本中最高子版本)
       linux kernel ver中 genphy_soft_reset()->phy_write(phydev, MII_BMCR, BMCR_RESET);

###########V2.2.11407 Description###########
1. 2022/12/12; 
code optimization: 
add .probe() callback to get chip mode and polling mode, 
avoiding to read chip mode every time in .read_status callback.

###########V2.2.10068 Description###########
1. 2022/10/21; updated to phy_write() in yt8821_suspend() & yt8821_resume()

###########V2.2.9888 Description###########
1. 2022/10/17; update yt8821(2.5G phy) phy driver

###########V2.2.8661 Description###########
1. 2022/08/19; Fixed issue #2955 YT8614 chip mode sgmii<->utp 处理 issue
2. 2022/08/19; Fixed Task  #2906 yt8512_led_init(): ret = ytphy_write_ext(phydev, YT8512_LED1_BT_ON_EN , val);
   update to ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED1 , val);

###########V2.2.7697 Description###########
1. 2022/07/08; Fixed issue #2679, add lock on yt phy_read_ext and ytphy_write_ext

###########V2.2.7290 Description###########
1. 2022/06/21; no updated except ver info

###########V2.2.6255 Description###########
1. 2022/05/10; 删除phy driver中关于yt8512的phy mode配置不甚合理部分

###########V2.2.6034 Description###########
1. 2022/04/26; added phy driver ver

###########V2.2.5949 Description###########
1. 2022/04/23; 增加 yt8010AS phy driver
2. 2022/03/23; 支持不同mdio bus上挂相同地址的phy设备
