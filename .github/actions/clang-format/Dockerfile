FROM alpine:3.13

RUN apk update \
  && apk add --no-cache --progress bash clang \
  && rm -rf /var/cache/apk/* 

COPY ./entrypoint.sh /entrypoint.sh

ENTRYPOINT ["bash","/entrypoint.sh"]
