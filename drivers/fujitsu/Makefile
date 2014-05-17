#
# Makefile for fujitsu devices that really don't fit anywhere else.
#

# FUJITSU:2011/12/13 ICS start
obj-y += proximity/
# FUJITSU:2011/12/13 ICS end

obj-y += fcsmd/
obj-y += fcfd/
obj-$(CONFIG_FUJITSU_COMPASS) += compass/
# FUJITSU:2011/07/25 del for I2CUART start
#obj-y += i2cuart/
# FUJITSU:2011/07/25 del for I2CUART end
obj-$(CONFIG_FUJITSU_WALKMOTION) += walkmotion/
obj-y += pipe/
obj-y += irda/
obj-y += memread/
obj-y += keypad/
# FUJITSU:2012-01-12 DN ICSH_001 start
obj-y += testmode/
# FUJITSU:2012-01-12 DN ICSH_001 end
# FUJITSU:2011/12/22 add extsd start
obj-$(CONFIG_MACH_F11SKY) += sd/
obj-$(CONFIG_MACH_F09D) += sd/
# FUJITSU:2011/12/22 add extsd end
obj-y += ftdtv_drv/
# FUJITSU:2011/08/02 add for STSCHK start
obj-y += stschk/
# FUJITSU:2011/08/02 add for STSCHK end
