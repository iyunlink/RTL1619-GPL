
is_rtl_drv_phy() {
	local phy=$1
	local drvp=`readlink /sys/class/ieee80211/$phy/device/driver 2>/dev/null`
	local drvprefix=`echo ${drvp##*/} | cut -c 1-4`
	[ "$drvprefix" = "rtl8" ] && return 0
	return 1
}

is_rtl_multi_phy() {
	local phy=$1
	is_rtl_drv_phy $phy || return 1

	local path=$(readlink -f /sys/class/ieee80211/${phy}/device)

	for d in "`ls -I $phy /sys/class/ieee80211`"; do
		local p=$(readlink -f /sys/class/ieee80211/${d}/device)
		is_rtl_drv_phy $d && [ "$p" = "$path" ] && return 0
	done

	return 1
}

check_rtl_phy_intf_name() {
	local phy=$1
	local if_idx=$2

	INTFS="`ls /sys/class/ieee80211/${phy}/device/net/`"
	if is_rtl_multi_phy $phy; then
		for intf in $INTFS; do
			local phyidx="`cat /sys/class/net/$intf/phy80211/index`"
			[ "${phy#phy}" = "$phyidx" ] && echo $intf && return 0
		done
	else
		if_idx=$((${if_idx:-0} + 1))
		ifname=`echo $INTFS | awk '{print $'$if_idx'}'`
		[ -n "$ifname" ] && echo $ifname && return 0
	fi
	return 1
}
