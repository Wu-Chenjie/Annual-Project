FROM python:3.10-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        git \
    && rm -rf /var/lib/apt/lists/*

COPY pyproject.toml README.md requirements.lock.txt requirements-dev.lock.txt ./
COPY core ./core
COPY simulations ./simulations
COPY scripts ./scripts
COPY schemas ./schemas
COPY maps ./maps
COPY cpp ./cpp
COPY config.py main.py ./

RUN python -m pip install --upgrade pip \
    && python -m pip install -r requirements-dev.lock.txt \
    && python -m pip install -e ".[dev,web]" \
    && cmake -S cpp -B cpp/build -G Ninja \
    && cmake --build cpp/build

CMD ["bash", "scripts/reproduce.sh", "--quick"]
