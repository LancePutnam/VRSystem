#!/bin/sh

# VRSystem dependencies install script

binary_exists(){
	command -v "$1" >/dev/null 2>&1;
}

files_exist(){
	ls -u "$@" >/dev/null 2>&1;
}

if binary_exists "apt-get"; then
	echo 'Found apt-get'
elif binary_exists "brew"; then
	echo 'Found Homebrew'
elif binary_exists "port"; then
	echo 'Found MacPorts'
elif uname -o | grep -q "Msys"; then

	# MSYS
	if files_exist /msys.bat; then
		echo 'Found MinGW / MSYS'
		if ! binary_exists "wget"; then
			echo "wget not found. Install with 'mingw-get install msys-wget'."
			exit
		fi
		if ! binary_exists "unzip"; then
			echo "unzip not found. Install with 'mingw-get install msys-unzip'."
			exit
		fi
		DESTDIR=/usr/local/
		#DESTDIR=local/
		DOWNLOAD="wget --no-check-certificate"
		install -d $DESTDIR/bin/ $DESTDIR/include/ $DESTDIR/lib/

	# MSYS2
	else
		echo 'Found MinGW-w64 / MSYS2'

		ARCH="mingw-w64-"$(uname -m)
		LIBS="openvr"
		PKGS=
		for L in $LIBS
		do
			PKGS=$PKGS" $ARCH-$L"
		done

		#echo $PKGS
		pacman -Syu
		#pacman -S $PKGS

		# Manual installation
		DESTDIR=/mingw64
		#install -d $DESTDIR/bin $DESTDIR/include $DESTDIR/lib

		if files_exist $DESTDIR/lib/SRanipal*; then
			echo 'Found SRanipal'
		else
			PKG=SRanipal_win_x64
			DIR=$PWD
			mkdir /tmp/$PKG
			cd /tmp/$PKG
				cp $DIR/ext/$PKG.zip $PKG.zip
				tar -xzf $PKG.zip
				#unzip -q $PKG -d $PKG
				install -d $DESTDIR/include/ViveSR
				cp include/* $DESTDIR/include/ViveSR
				cp bin/* $DESTDIR/bin
				cp lib/* $DESTDIR/lib
			cd $DIR
		fi

	fi
else
	echo 'Error: No suitable package manager found.'
	echo 'Error: Install apt-get, MacPorts, or Homebrew and try again.'
fi
