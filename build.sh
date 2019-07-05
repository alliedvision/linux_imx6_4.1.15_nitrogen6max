#!/bin/bash
#==============================================================================
# Copyright (C) 2019 Allied Vision Technologies.  All Rights Reserved.
#
# Redistribution of this file, in original or modified form, without
# prior written consent of Allied Vision Technologies is prohibited.
#
#------------------------------------------------------------------------------
#
# File:         -build.sh
#
# Description:  -bash script for building kernel
#
#------------------------------------------------------------------------------
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
# NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
# DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#==============================================================================

# ========================================================= colors
NC='\033[0m'
BLACK='\033[0;30m'
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
LIGHT_GRAY='\033[0;37m'
DARK_GRAY='\033[1;30m'
LIGHT_RED='\033[1;31m'
LIGHT_GREEN='\033[1;32m'
YELLOW='\033[1;33m'
LIGHT_BLUE='\033[1;34m'
LIGHT_PURPLE='\033[1;35m'
LIGHT_CYAN='\033[1;36m'
WHITE='\033[1;37m'

# ========================================================= logging function
log()
{
    if [[ ($1 == "info") ]]
    then
        echo -e "${WHITE}${2}${NC}"

    elif [[ ($1 == "success") ]]
    then
        echo -e "${GREEN}Success!${2}${NC}"

    elif [[ ($1 == "failed") ]]
    then
        echo -e "${RED}Fail!${NC}"

    elif [[ ($1 == "hint") ]]
    then
        echo -e "${YELLOW}${2}${NC}"

    elif [[ ($1 == "error") ]]
    then
        echo -e "${RED}${2}!${NC}"
    fi
}

# ========================================================= default parameters
export KERNEL_SRC=$PWD
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export INSTALL_MOD_PATH=${KERNEL_SRC}/ubuntunize/linux-staging

# ========================================================= usage function
usage() {
	echo -e "This script is used for kernel configuration and building."	
	echo -e "--help, -h:\t Display help"
	echo -e "--clean:\t Clean the object files"
	echo -e "--distclean:\t Clean complete object files and configurations"
	echo -e "--defconfig:\t Create default configuration"
	echo -e "--config:\t Open the kernel drivers config (make menuconfig)"
	echo -e "--build:\t Build the kernel, dtb, and tar file"
#	echo -e "To build the kernel, dtb, and tar file for production mode, run,  ./build_script --production_build"
	echo -e "--auto:\t\t Build everything"
	echo -e "--cross_compile: Optional path the crosscompiler binary. Usage e.g.: ./build.sh --build --cross_compile /home/abc/ ... /bin/arm-linux-gnueabihf-"
	echo -e "--debug: Optional parameter to build driver modules with DEBUG flag (enable pr_debug() output)"
	exit 1
}
# ========================================================= enable/disable debug build function
FLAG_DEBUG=false

debugEnable()
{
	log info "Build kernel with DEBUG flag"
	cp drivers/media/platform/mxc/capture/Makefile.debug drivers/media/platform/mxc/capture/Makefile
	FLAG_DEBUG=true
}

debugDisable()
{
	log info "Swap to original Makefile"
	cp drivers/media/platform/mxc/capture/Makefile.release drivers/media/platform/mxc/capture/Makefile
}

if [[ ( $1 == "--help") || ( $1 == "-h") ]]
	then
	usage
fi	

if [[ ( $2 == "--cross_compile") ]]
	then
	export CROSS_COMPILE=$3
elif [[ ( $2 == "--debug") ]]
	then
		debugEnable
fi

if [[ ( $4 == "--debug") ]]
	then
		debugEnable
fi

# ========================================================= dump parameters
log info "The following variables have been exported:"
log hint "KERNEL_SRC=${KERNEL_SRC}"	
log hint "ARCH=${ARCH}"	
log hint "CROSS_COMPILE=${CROSS_COMPILE}"
log hint "INSTALL_MOD_PATH=${INSTALL_MOD_PATH}"

if [[ ( $1 == "--defconfig") ]]
	then
	make distclean
	make nitrogen6x_alliedvision_defconfig
	exit 0
fi

if [[ ( $1 == "--auto") ]]
	then
	# clean dir 
	rm -rf $INSTALL_MOD_PATH
	mkdir $INSTALL_MOD_PATH
	make distclean
	make nitrogen6x_alliedvision_defconfig
	if make zImage modules dtbs -j4
	then
		log success " Kernel build done."
	else
		log failed
		exit 1
        fi

	if make -C ubuntunize tarball
	then
		log success " Tarball created."
	else
		log failed
		exit 1
        fi

	cd $KERNEL_SRC/kernel-module-imx-gpu-viv/
	if make
	then
		log success " Galcore GPU driver build done."
	else
		log failed
		exit 1
        fi	

	make modules_install
	make -C $KERNEL_SRC/ubuntunize targz

	cd $KERNEL_SRC

	if [[ ( $FLAG_DEBUG == "true" ) ]]
	then
		debugDisable
	fi

	sync
	exit 0
fi

if [[ ( $1 == "--distclean") ]]
	then
	make distclean
	exit 0
fi

if [[ ( $1 == "--clean") ]]
	then
	# clean dir	
	rm -rf $INSTALL_MOD_PATH
	mkdir $INSTALL_MOD_PATH
	make clean
	exit 0
fi

if [[ ( $1 == "--config") ]]
	then
	make menuconfig
	exit 0
fi


if [[ ( $1 == "--build") ]]
	then
	# clean dir	
	rm -rf $INSTALL_MOD_PATH
	mkdir $INSTALL_MOD_PATH
	if make zImage modules dtbs -j4
	then
		log success " Kernel build done."
	else
		log failed
		exit 1
        fi

	if make -C ubuntunize tarball
	then
		log success " Tarball created."
	else
		log failed
		exit 1
        fi

	cd $KERNEL_SRC/kernel-module-imx-gpu-viv/
	if make
	then
		log success " Galcore GPU driver build done."
	else
		log failed
		exit 1
        fi	

	make modules_install
	make -C $KERNEL_SRC/ubuntunize targz

	cd $KERNEL_SRC
	if [[ ( $FLAG_DEBUG == "true" ) ]]
	then
		debugDisable
	fi

	sync
	exit 0
fi

<<comment
if [[ ( $1 == "--production_build") ]]
	then
	make zImage modules dtbs -j4 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
	echo -e "${YELLOW} Kernel build done! ${NC}"
	make -C ubuntunize tarball_production ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
	echo -e "${YELLOW} Tar done! ${NC}"
	exit 0
fi
comment

usage
