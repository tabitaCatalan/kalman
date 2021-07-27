# Changelog
Los cambios notables al proyecto serán documentados aquí. Esta es la primera vez que intento mantener un proyecto bien documentado, y comencé cuando ya había avanzado bastante, por lo que, este *change log* no es muy detallado en las versiones más antiguas. La idea es ir aprendiendo a medida que avanzo. 

El formato está basado en su mayor parte en [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), y el proyecto intenta adherirse al [Semantic Versioning](https://semver.org/spec/v2.0.0.html), aunque aún tengo problemas para seguirlo.

## Trabajo futuro 
- Añadir *smoother* al paquete. 
- Mejorar el diseño de `EnKF` para manejar mejor la inflación de varianza.
- Eliminar `NLUnknowInput` y hacer una forma más fácil de trabajar con sistemas con inputs desconocidos.
- Mejorar el diseño del código para graficar, para manejar distintos modelos.
- Hacer notebooks de los archivos de ejemplo `SDE`.

## Unreleased

## Versión `v0.9.0` - 2021-07-27 
Hice muchos cambios sin preocuparme del versionamiento, así que este incluye muchas cosas. La documentación no está al día con esta versión. 
### Cambios de interfaces y constructores 
- ![Changed][badge-changed] Se cambia el modelo lineal básico; antes se tenía un vector ``F``, el cual era ponderado por un valor dado por una normal. Ahora se considera como una matriz, multiplicada por un vector aleatorio que distribuye normal multivariado (generalmente independientes).
- ![Changed][badge-changed] Se cambia la interfaz a los updaters, ya no reciben el ruido de parte del `KalmanIterator`, si no que lo almacenan dentro. Se modifica `EnKF` en consecuencia.
- ![Changed][badge-changed] `NLUpdater` ahora recibe una función `F(x)` que retorna una matriz.
- ![Added][badge-added] Se crea el tipo abstracto `LinearizableUpdater`, que engloba a `SimpleLinearUpdater` y `NLUpdater`, pero no a `ODEUpdater`.
- ![Changed][badge-changed] Cambia el constructor de `ObservableSystem`, ahora recibe un Array 2d como observación. 
- ![Changed][badge-changed] Cambia la interfaz de `Updater`, ahora `forecast` devuelve un `ComponentArray`.
- ![Changed][badge-changed] Se agrega `forecast_with_error` a `Updater`, para ser usado por `EnKF`. 

### Nuevas *Features*
- ![Added][badge-added] `EnKF` trabaja ahora con varianza inflada, pero fue *hard-wired*. Debe ser corregido en futuras versiones.
- ![Add][badge-added] ![Change][badge-changed] `KalmanIterator` ahora puede aplicar un filtro *low-pass* a los resultados del análisis. Esto cambia la interfaz de `KalmanIterator`.
- ![Add][badge-added] Ahora Se chequea numéricamente la observabilidad local del sistema en cada paso al correr el filtro. Esto no funciona con `Updater`s que no sean de tipo `LinearizableUpdater`.
- ![Added][badge-added] Observador no lineal `NLObserver`. 
- ![Added][badge-added] Métodos para graficar `FilteredSeries` junto a `EnsambleStoring`. 
- ![Added][badge-added] Se agrega `ContinuousDiscretMomentum` para facilitar el manejo de sistemas continuo discretos a `ODEUpdater`. Incluye `ExtendedMomentum` para trabajar con EKF y también `UnscentedMomentum`, que permite trabajar con UKF. 
- ![Added][badge-added] Se agrega un código de ejemplo de *smoothing* para sistema continuo-discretos, pero aún no forma parte del paquete. 

### Correción de errores 
- ![Fixed][badge-fixed] Se corrigen errores en `ODEForecaster`; no estaba resolviendo la ecuación de momentos de forma acoplada, entre otras cosas.
- ![Fixed][badge-fixed] `EnKF` y `KalmanIterator` estaban mutando los arrays de condiciones iniciales que se le daban. 
- ![Fixed][badge-fixed] Había un error al generar ruido gaussiano (se usaba `dt^2` en lugar de `sqrt(dt)`).

### Ejemplos 
- ![Added][badge-added] Nuevos ejemplos con datos reales y sintéticos (códigos `SDE`). Solo archivos en bruto, mejorar a futuro como notebooks.
- ![Changed][badge-changed] Se cambia la organización de los ejemplos, ahora las definiciones comunes están en un archivo `common_defs.jl`.

### Otros 
- ![Changed][badge-changed] El modelo principal de los ejemplos ahora considera Recuperados. Los plots que agregan label a los compartimientos ahora lo consideran. Esto debería corregirse a futuro con una forma mejor de manejar los gráficos de distintos modelos.
- ![Changed][badge-changed] Se mejora la estabilidad del análisis, ahora los cálculos deberían asegurar simetría de la matriz de covarianza.
- ![Deprecated][badge-deprecated] `NLUnknowInput` se dejará de usar. Sus funcionalidades pueden obtenerse resolviendo un sistema ampliado. En un futuro se harán herramientas para hacer esto más fácil. 

## Versión `v0.8.0` - 2021-04-23
- ![Added][badge-added] Se agrega `ODEForecaster`, un nuevo `KalmanUpdater` que permite hacer una predicción del siguiente estado resolviendo las ecuaciones de momento del filtro de Kalman continuo-discreto.
- ![Changed][badge-changed] Se cambia los constructores de varios `KalmanObserver` y `KalmanUpdater`, para permitir al usuario definir una función de integridad. 
- ![Changed][badge-changed] Se cambia la interfaz de `KalmanUpdater` para asignarle la función de hacer el *forecast* a partir de `hatx` y `hatP`. Sujeto a cambios, hay que revisar teóricamente.
- ![Added][badge-added] Se agrega `SimpleRK4`, un `Discretizer` de tipo `RK4` que solo requiere una función `f` (y no sus derivadas).
- ![Fixed][badge-fixed] Se corrige un error en `Measurements`, entregaba observaciones escalares en lugar de vectores. (Esto debió ser una versión nueva).

## Versión `v0.7.0` - 2021-04-17
- ![Added][badge-added] Se agrega el tipo abstracto `ObservableSystem`, que permite definir una interfaz común para el ingreso de datos al sistema, ya sea almacenando un estado interno mediante la estructura `InnerState`, o guardando solo observaciones mediante `Measurements`.
- ![Changed][badge-changed] `LinearKalmanIterator` ahora guarda un `ObservableSystem`. Cambió la forma de actualizar y observar ese sistema.
- ![Added][badge-added] Se agrega la página `Systems` a la documentación, explicando la interfaz de `ObservableSystem`. 
- ![Changed][badge-changed] Se actualiza la documentación de `KalmanObserver` y `KalmanIterator`.

## Versión `v0.6.0`- 2021-04-16
- ![Added][badge-added] Nueva estructura `EnsamblesStoring` para almacenar los ensambles durante la iteración de `EnKF`. 
- ![Added][badge-added] Receta para graficar `EnsamblesStoring`.

## Versión `v0.5.1` - 2021-04-16
- ![Fixed][badge-fixed] `EnKF` no actualizaba al `Updater` para ahorrar trabajo, pero esto sí era necesario (sobre todo con el updater que incluye un input desconocido). Se corrigió. Se necesita un updater más liviano.
- ![Fixed][badge-fixed] Se agregan restricciones de integridad a los `KalmanUpdater` `NLUpdater` y `SimpleLinearUpdater`, pero de una mala forma, hay que corregirlo, lo que cambiará la interfaz de `KalmanUpdater`.

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
