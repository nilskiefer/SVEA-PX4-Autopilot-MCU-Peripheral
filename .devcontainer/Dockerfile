FROM espressif/idf:latest

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    zsh \
    zsh-autosuggestions \
    zsh-syntax-highlighting \
    git \
    curl \
    ca-certificates \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN export RUNZSH=no CHSH=no KEEP_ZSHRC=yes \
 && sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" \
 && sed -i 's/^ZSH_THEME=.*/ZSH_THEME="robbyrussell"/' /root/.zshrc \
 && sed -i 's/^plugins=.*/plugins=(git)/' /root/.zshrc \
 && echo 'source /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh' >> /root/.zshrc \
 && echo 'source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh' >> /root/.zshrc \
 && echo 'source /opt/esp/idf/export.sh' >> /root/.zshrc \
 && echo "alias ll='ls -alF'" >> /root/.zshrc \
 && echo 'source /opt/esp/idf/export.sh' >> /root/.bashrc
