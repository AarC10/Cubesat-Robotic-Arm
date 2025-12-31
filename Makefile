.PHONY: up down exec logs build

up:
	LOCAL_UID=$$(id -u) LOCAL_GID=$$(id -g) docker compose up --build -d

down:
	docker compose down

exec:
	docker compose exec ros bash

logs:
	docker compose logs -f ros

build:
	docker compose build

