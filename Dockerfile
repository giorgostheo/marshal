FROM ubuntu:22.10
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
# RUN update-alternatives --remove-all gcc 
# RUN update-alternatives --remove-all g++
RUN apt-get install -y \
    build-essential \
    software-properties-common \
    curl \
    make \
    cmake \
    unzip \
    libsqlite3-dev \ 
    python3-dev \
    libproj-dev proj-data proj-bin libgeos-dev\
    sqlite3 \
    pkg-config \
    libclang-dev \
    # clang-12 \
    python3-pip


# RUN add-apt-repository ppa:deadsnakes/ppa
# RUN apt-get update

# RUN apt install python3.9

WORKDIR /opt
# RUN pip install torch==2.0.0
# RUN apt-get update
# RUN python3 -c "import torch; print(torch.__path__[0])"
# RUN pkg-config --modversion proj

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

ENV LIBTORCH_URL=https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcpu.zip
RUN curl -fsSL --insecure -o libtorch.zip  $LIBTORCH_URL \
    && unzip -q libtorch.zip \
    && rm libtorch.zip
# # RUN ls /opt/libtorch
ENV LIBTORCH=/opt/libtorch
# ENV LIBTORCH=/usr/local/lib/python3.10/dist-packages/torch
ENV LD_LIBRARY_PATH=${LIBTORCH}/lib

WORKDIR /app
COPY . .
# RUN cargo run --release > result.txt