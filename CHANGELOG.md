# Changelog
Los cambios notables al proyecto serán documentados aquí. Esta es la primera vez que intento mantener un proyecto bien documentado, y comencé cuando ya había avanzado bastante, por lo que, este *change log* no es muy detallado en las versiones más antiguas. La idea es ir aprendiendo a medida que avanzo. 

El formato está basado en su mayor parte en [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), y el proyecto se adhiere al [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## Versión `v0.2.0` - 2021-04-08
- ![Added][badge-added] Es posible usar filtro de kalman para mantener un sistema interno lineales o no lineal, el cual puede ser observado con un un observador lineal. Es posible averiguar un input desconocido (un número, no un vector). 
- ![Added][badge-added] Se agrega documentación (aun en desarrollo) para `Observer`, `Iterator`, `Updater`, `Discretizer`.
- ![Added][badge-added] Se agrega a la documentación una descripción muy corta de lo que es el Filtro de Kalman (faltan las ecuaciones).
- ![Added][badge-added] Se agrega un tutorial: Filtro de Kalman Extendido con input desconocido.

[badge-removed]: https://img.shields.io/static/v1?label=&message=Removed&color=critical&style=flat-square
[badge-added]: https://img.shields.io/static/v1?label=&message=Added&color=brightgreen&style=flat-square
[badge-deprecated]: https://img.shields.io/static/v1?label=&message=Deprecated&color=orange&style=flat-square
[badge-enhancement]: https://img.shields.io/static/v1?label=&message=Enhancement&color=blue&style=flat-square 
[badge-fixed]: https://img.shields.io/static/v1?label=&message=Fixed&color=blueviolet&style=flat-square 
[badge-experimental]: https://img.shields.io/static/v1?label=&message=Experimental&color=lightgrey&style=flat-square

<!-- Ideas de badges 
[badge-breaking]: https://img.shields.io/badge/BREAKING-red.svg
[badge-deprecation]: https://img.shields.io/badge/deprecation-orange.svg
[badge-feature]: https://img.shields.io/badge/feature-green.svg
[badge-enhancement]: https://img.shields.io/badge/enhancement-blue.svg
[badge-bugfix]: https://img.shields.io/badge/bugfix-purple.svg
[badge-security]: https://img.shields.io/badge/security-black.svg
[badge-experimental]: https://img.shields.io/badge/experimental-lightgrey.svg
[badge-maintenance]: https://img.shields.io/badge/maintenance-gray.svg
-->