#
# This file is the custom-files recipe.
#

SUMMARY = "Simple custom-files application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

#SRC_URI = "file://custom-files.c \
#	   file://Makefile \
#		  "
#
#S = "${WORKDIR}"
#
#do_compile() {
#	     oe_runmake
#}
#
#do_install() {
#	     install -d ${D}${bindir}
#	     install -m 0755 custom-files ${D}${bindir}
#}

S = "${TOPDIR}/../project-spec/meta-user/recipes-apps/custom-files/files/custom-files"

do_install() {
             install -d ${D}/etc/dropbear
             install -m 0644 ${S}/etc/dropbear/dropbear_rsa_host_key ${D}/etc/dropbear

             install -d ${D}/etc/rcS.d
             install -m 0755 ${S}/etc/rcS.d/S45password ${D}/etc/rcS.d

             install -d ${D}/etc/rc5.d
             install -m 0755 ${S}/etc/rc5.d/S25time     ${D}/etc/rc5.d

             install -d ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_dsa_key         ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_dsa_key.pub     ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_ecdsa_key       ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_ecdsa_key.pub   ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_ed25519_key     ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_ed25519_key.pub ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_key             ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_key.pub         ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_rsa_key         ${D}/etc/ssh
             install -m 0644 ${S}/etc/ssh/ssh_host_rsa_key.pub     ${D}/etc/ssh

             install -d ${D}/home/root/
             install -m 0755 ${S}/home/root/sourceme.sh ${D}/home/root

             install -d ${D}/home/root/.ssh
             install -m 0644 ${S}/home/root/.ssh/authorized_keys ${D}/home/root/.ssh
             install -m 0644 ${S}/home/root/.ssh/known_hosts     ${D}/home/root/.ssh

             install -m 0755 ${S}/mount_nfs.sh ${D}
             install -m 0755 ${S}/sourceme.sh  ${D}
}

FILES_${PN} += "/etc/dropbear/dropbear_rsa_host_key \
            /etc/rcS.d/S45password \
            /etc/rc5.d/S25time \
            /etc/ssh/ssh_host_dsa_key \
            /etc/ssh/ssh_host_dsa_key.pub \
            /etc/ssh/ssh_host_ecdsa_key \
            /etc/ssh/ssh_host_ecdsa_key.pub \
            /etc/ssh/ssh_host_ed25519_key \
            /etc/ssh/ssh_host_ed25519_key.pub \
            /etc/ssh/ssh_host_key \
            /etc/ssh/ssh_host_key.pub \
            /etc/ssh/ssh_host_rsa_key \
            /etc/ssh/ssh_host_rsa_key.pub \
            /home/root/sourceme.sh \
            /home/root/.ssh/authorized_keys \
            /home/root/.ssh/known_hosts \
            /mount_nfs.sh \
            /sourceme.sh "
