#! /bin/bash

ROOT="$( cd "$(dirname "$0")" ; pwd -P )"
mkdir -p $ROOT/toolchain
pushd ./toolchain
if [ "$(uname)" == "Darwin" ]; then
    sudo easy_install pip
    wget -qO- https://dl.espressif.com/dl/xtensa-esp32-elf-osx-1.22.0-80-g6c4433a-5.2.0.tar.gz | tar xvz -
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt-get install -y \
        gcc \
        git \
        wget \
        make \
        libncurses-dev \
        flex \
        bison \
        gperf \
        python \
        python-pip \
        python-setuptools \
        python-serial \
        python-cryptography \
        python-future \
        python-pyparsing \
        python-pyelftools
    wget -qO- https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz | tar xvz -
fi
git clone --recursive https://github.com/espressif/esp-idf.git
export "PATH=$ROOT/toolchain/xtensa-esp32-elf/bin:$PATH"
export "IDF_PATH=$ROOT/toolchain/esp-idf"
echo "export PATH=$ROOT/toolchain/xtensa-esp32-elf/bin:\$PATH" >> $HOME/.bash_profile
echo "export IDF_PATH=$ROOT/toolchain/esp-idf" >> $HOME/.bash_profile
popd

python -m pip install --user -r $IDF_PATH/requirements.txt
