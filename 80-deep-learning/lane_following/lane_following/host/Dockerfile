FROM tensorflow/tensorflow:latest-devel-gpu

MAINTAINER David Uhm <david.uhm@lge.com>

RUN apt-get update
RUN apt-get install -y python-opencv vim

COPY ./requirements.txt /requirements.txt

RUN pip install --upgrade pip
RUN pip install -r /requirements.txt

EXPOSE 8888

VOLUME [".:/notebooks"]

WORKDIR /notebooks

CMD [ "/run_jupyter.sh", "--allow-root" ]
