# Changelog
Los cambios notables al proyecto serán documentados aquí. Esta es la primera vez que intento mantener un proyecto bien documentado, y comencé cuando ya había avanzado bastante, por lo que, este *change log* no es muy detallado en las versiones más antiguas. La idea es ir aprendiendo a medida que avanzo. 

El formato está basado en su mayor parte en [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), y el proyecto intenta adherirse al [Semantic Versioning](https://semver.org/spec/v2.0.0.html), aunque aún tengo problemas para seguirlo.

## Trabajo futuro 
- Agregar un `KalmanObserver` que permita incorporar datos externos.

## Unreleased

## Versión `v0.6.0`- 2021-04-16
- ![Added][badge-added] Nueva estructura `EnsamblesStoring` para almacenar los ensambles durante la iteración de `EnKF`. 
- ![Added][badge-added] Receta para graficar `EnsamblesStoring`.

## Versión `v0.5.1` - 2021-04-16
- ![Fixed][badge-fixed] `EnKF` no actualizaba al `Updater` para ahorrar trabajo, pero esto sí era necesario (sobre todo con el updater que incluye un input desconocido). Se corrigió. Se necesita un updater más liviano.
- ![Fixed][badge-fixed] Se agregan restricciones de integridad a los `KalmanUpdater` `NLUpdater` y `SimpleLinearUpdater`, pero de una mala forma, hay que corregirlo, lo que cambiará la interfaz de `KalmanUpdater`.
- 
## Versión `v0.5.0` - 2021-04-13
- ![Added][badge-added] Se agrega la estructura `FilteredSeries` para almacenar los resultados de la iteración de Kalman.
- ![Added][badge-added] Se añade `RecipesBase` a las dependencias. 
- ![Added][badge-added] Se crea una receta para graficar `FilteredSeries`, para el caso en que sus resultados corresponden al modelo SEIIcI con input desconocido.
- ![Fixed][badge-fixed] Se corrige un error en `LinearObserver`; modificaba el vector `x0` de condiciones iniciales, lo que causaba problemas al correrlo por segunda vez.
- ![Fixed][badge-fixed] Se agregan restricciones de integridad a los estados de `LinearObserver`. Esto permite que no se obtengan valores sin sentido físico ( valores negativos en una EDO epidemiológica) tanto en los estados como en las observaciones. Se requiere más generalidad.
- ![Fixed][badge-fixed] Se corrige un error en el ruido de las EDOs, se está usando un valor constante pero debería ser dependiente del tamaño del paso (Normal(0, dt)). Ver método de Euler-Maguyama.

## Versión `v0.4.0` - 2021-04-09
- ![Added][badge-added] Se agrega un iterador `EnKF` que permite trabajar con Filtro de Kalman por ensambles.
- ![Added][badge-added] Se agrega una función `kalman_size(observer::KalmanObserver)` que devuelve el tamaño de la matriz de Kalman asociada.

## Versión `v0.3.0` - 2021-04-08
- ![Changed][badge-changed] Mayor modularidad al cambiar el comportamiento (y la interfaz) de `KalmanObserver`. `KalmanIterator` no necesita conocer el sistema real que se intenta *trackear*, se deja que `KalmanObserver` se encargue de él, por lo que ahora tener un sistema real interno es opcional. Esto abre la puerta a trabajar con sistemas físicos de los que únicamente se conocen los datos de mediciones.
- ![Fixed][badge-fixed] Se corrige el ejemplo en la documentación del Filtro de Kalman Extendido con Input desconocido. Ahora sí aparecen los gráficos.

## Versión `v0.2.0` - 2021-04-08
- ![Added][badge-added] Es posible usar filtro de kalman para mantener un sistema interno lineales o no lineal, el cual puede ser observado con un un observador lineal. Es posible averiguar un input desconocido (un número, no un vector). 
- ![Added][badge-added] Se agrega documentación (aun en desarrollo) para `Observer`, `Iterator`, `Updater`, `Discretizer`.
- ![Added][badge-added] Se agrega a la documentación una descripción muy corta de lo que es el Filtro de Kalman (faltan las ecuaciones).
- ![Added][badge-added] Se agrega un tutorial: Filtro de Kalman Extendido con input desconocido.

[badge-removed]: https://img.shields.io/static/v1?label=&message=Removed&color=critical&style=flat-square
[badge-added]: https://img.shields.io/static/v1?label=&message=Added&color=brightgreen&style=flat-square
[badge-deprecated]: https://img.shields.io/static/v1?label=&message=Deprecated&color=orange&style=flat-square
[badge-changed]: https://img.shields.io/static/v1?label=&message=Changed&color=blue&style=flat-square 
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
