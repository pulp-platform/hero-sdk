#!/bin/bash

mkdir openembedded && cd ./openembedded
export OE_HOME=`pwd`
git clone git://git.linaro.org/openembedded/jenkins-setup.git
cd $OE_HOME/jenkins-setup
git checkout release-${OE_RELEASE}
cd $OE_HOME
#sudo jenkins-setup/pre-build-root-install-dependencies.sh

# refresh the repo tool, the corresponding command in the above fails to install it because of a broken link
test -d ~/bin || mkdir -p ~/bin
curl https://storage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
chmod a+x ~/bin/repo
