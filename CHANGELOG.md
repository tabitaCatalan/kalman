# Changelog
Los cambios notables al proyecto serán documentados aquí. Esta es la primera vez que intento mantener un proyecto bien documentado, y comencé cuando ya había avanzado bastante, por lo que, este *change log* no es muy detallado en las versiones más antiguas. La idea es ir aprendiendo a medida que avanzo. 

El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), y el proyecto se adhiere al [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2021-04-08
### Added 
- Es posible usar filtro de kalman para mantener un sistema interno lineales o no lineal, el cual puede ser observado con un un observador lineal. Es posible averiguar un input desconocido (un número, no un vector). 
- Se agrega documentación (aun en desarrollo) para `Observer`, `Iterator`, `Updater`, `Discretizer`.
- Se agrega a la documentación una descripción muy corta de lo que es el Filtro de Kalman (faltan las ecuaciones).
- Se agrega un tutorial: Filtro de Kalman Extendido con input desconocido.