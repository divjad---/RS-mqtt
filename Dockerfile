FROM python:3.8-slim-buster
LABEL authors="davidtrafela"

ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

WORKDIR /python-docker

COPY req.txt .
RUN pip3 install -r req.txt

COPY . .

CMD [ "python3", "-m" , "flask", "run", "--host=0.0.0.0"]