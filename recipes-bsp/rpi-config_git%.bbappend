
 do_append_i2c_config() {
    CONFIG_FILE="${B}/rootfs/boot/config.txt"
    echo "dtparam=i2c1=on" >> ${CONFIG_FILE}
}

addtask append_i2c_config before do_install
