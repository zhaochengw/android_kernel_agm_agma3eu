#! /bin/bash

oldext="o"
newext="obj"

OUT=kout

# process the sensors
#cp ./$OUT/drivers/sensors/sensors_class.o  ./drivers/sensors/sensors_class.obj
#cp -r ./$OUT/drivers/sensors/HS_SNS_DRV   ./drivers/sensors/
#rm -rf ./drivers/sensors/HS_SNS_DRV/built-in.o
#rm -rf ./drivers/sensors/HS_SNS_DRV/modules.builtin
#rm -rf ./drivers/sensors/HS_SNS_DRV/modules.order

#rm -rf ./drivers/sensors/HS_SNS_DRV/*.c
#rm -rf ./drivers/sensors/HS_SNS_DRV/*.h
#rm -rf ./drivers/sensors/sensors_class.c

#cd ./drivers/sensors/HS_SNS_DRV/
#dir=$(eval pwd)

#for file in $(ls $dir | grep -E "\.$oldext$")
#	do
#	name=$(ls $file | cut -d. -f1)
#	mv $file ${name}.$newext
#	echo "$name.$oldext ====> $name.$newext"
#	done

#cd ../../../
#rm -rf ./drivers/sensors/*.o
#rm -rf ./drivers/sensors/HS_SNS_DRV/*.o

#process the his feature
rm -rf ./drivers/soc/his/hisresume.c

#cp ./$OUT/drivers/soc/his/bootinfo.o  ./drivers/soc/his/bootinfo.obj
#rm -rf ./drivers/soc/his/bootinfo.c

rm -rf ./drivers/soc/his/his_serial_control.c

cp ./$OUT/drivers/gpio/gpio_unused.o ./drivers/gpio/gpio_unused.obj
rm -rf ./drivers/gpio/gpio_unused.c

cp ./$OUT/drivers/soc/his/hisfuse.o ./drivers/soc/his/hisfuse.obj
rm -rf ./drivers/soc/his/hisfuse.c

cp ./$OUT/drivers/soc/his/his_boot_log.o ./drivers/soc/his/his_boot_log.obj
rm -rf ./drivers/soc/his/his_boot_log.c

cp ./$OUT/drivers/soc/his/his_emmc_ops.o ./drivers/soc/his/his_emmc_ops.obj
rm -rf ./drivers/soc/his/his_emmc_ops.c

cp ./$OUT/drivers/soc/his/his_debug_control_node.o ./drivers/soc/his/his_debug_control_node.obj
rm -rf ./drivers/soc/his/his_debug_control_node.c

cp ./$OUT/drivers/soc/his/his_no_sd_ramdump.o ./drivers/soc/his/his_no_sd_ramdump.obj
rm -rf ./drivers/soc/his/his_no_sd_ramdump.c

#cp ./$OUT/drivers/soc/his/subsys_err_report.o ./drivers/soc/his/subsys_err_report.obj
#rm -rf ./drivers/soc/his/subsys_err_report.c

cp ./$OUT/drivers/soc/his/productinfo.o ./drivers/soc/his/productinfo.obj
rm -rf ./drivers/soc/his/productinfo.c

