
do_deploy:append() {
	 echo "# Enable I2C" >>$CONFIG
        echo "dtparam=i2c1=on" >>$CONFIG
        echo "dtparam=i2c_arm=on" >>$CONFIG
}
