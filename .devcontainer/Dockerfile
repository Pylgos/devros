# ROS 2 Jazzy with Rust development environment
FROM ros:jazzy-ros-base-noble

# Install basic development tools
RUN apt-get update && apt-get install -y \
  build-essential \
  clang \
  cmake \
  curl \
  git \
  libssl-dev \
  lld \
  lldb \
  pkg-config \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ssh \
  && curl -fsSL https://deb.nodesource.com/setup_24.x | bash - \
  && apt-get install -y nodejs \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Set bash as default shell
ENV SHELL=/bin/bash
RUN ln -sf /bin/bash /bin/sh || true

# Configure passwordless sudo for the ubuntu user
RUN printf '%s\n' "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/99_ubuntu_nopasswd \
  && chmod 0440 /etc/sudoers.d/99_ubuntu_nopasswd

# Install gemini-cli globally
RUN npm install -g @google/gemini-cli

# Install lazygit
RUN LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" | grep -Po '"tag_name": *"v\K[^"]*') && \
  curl -Lo lazygit.tar.gz "https://github.com/jesseduffield/lazygit/releases/download/v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz" && \
  tar xf lazygit.tar.gz lazygit && \
  install lazygit -D -t /usr/local/bin/

# runner user similar to hosted runners
RUN set -eux; \
    adduser --disabled-password --gecos "" --uid 1001 runner && \
    groupadd docker --gid 123 && \
    usermod -aG sudo runner && \
    usermod -aG docker runner && \
    echo "%sudo ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers && \
    echo "Defaults env_keep += \"DEBIAN_FRONTEND\"" >> /etc/sudoers

ENV CONTAINER=true
WORKDIR /home/runner

USER ubuntu

# Install Rust with rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable

# Add cargo to PATH
ENV PATH="/home/ubuntu/.cargo/bin:${PATH}"

# Install Rust components
RUN rustup component add rustfmt clippy rust-analyzer

# Install claude code
RUN curl -fsSL https://claude.ai/install.sh | bash
ENV PATH="/home/ubuntu/.local/bin:${PATH}"

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
