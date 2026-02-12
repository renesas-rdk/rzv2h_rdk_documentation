# -------------------------------------------------------------------
# Dockerfile for building RZV2H RDK documentation
# -------------------------------------------------------------------
FROM ubuntu:noble

ARG user=ubuntu
ARG uid=1000
ARG arch=amd64

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash

# Delete user if it exists (Ubuntu Noble has default 'ubuntu' user)
RUN if id -u $uid >/dev/null 2>&1; then userdel `id -un $uid`; fi

# Base dependencies
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        git-all \
        git-lfs \
        graphviz \
        locales \
        make \
        python3-pip \
        pandoc \
        ruby-full \
        curl \
        sudo \
        openssh-server \
        vim \
        ca-certificates \
        libenchant-2-2 \
        aspell \
        aspell-en && \
    rm -rf /var/lib/apt/lists/*

# HTML build dependencies
COPY requirements.txt constraints.txt ./

RUN pip3 install --no-cache-dir \
    --no-warn-script-location \
    --break-system-packages \
    -r requirements.txt -c constraints.txt

# AsciiDoc PDF toolchain for scripts/convert_pdf.sh
RUN gem install --no-document \
        asciidoctor \
        asciidoctor-pdf \
        asciidoctor-pdf-cjk

# Set locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create user
RUN adduser --disabled-password --shell /bin/bash \
    --gecos '' $user
RUN usermod -aG sudo $user && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV HOME=/home/$user
ENV PATH=/home/$user/.local/bin:$PATH
WORKDIR /tmp/doc_repository
