FROM debian:bookworm-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libeigen3-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN cmake -S . -B build -DICARUS_WITH_RERUN=OFF -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --target IcarusStream --parallel

FROM node:22-bookworm-slim
WORKDIR /app
COPY --from=build /src/build/IcarusStream /app/IcarusStream
COPY listen/server.mjs listen/case.mjs /app/
ENV PORT=8080
ENV ICARUS_STREAM=/app/IcarusStream
EXPOSE 8080
CMD ["node", "server.mjs"]
