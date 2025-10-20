## Taller Lazo Cerrado

### Ejercicio 1
Sean $x_{current}, y_{current}, a_{current}$ la posición y orientación actuales del robot, y $x_{goal}, y_{goal}, a_{goal}$ la posición y orientación objetivo.

Sean:
- $dx = x_{goal} - x_{current}$
- $dy = y_{goal} - y_{current}$
- $\theta = a_{goal} - a_{current}$

Definimos:
- $\rho = \sqrt{dx^2 + dy^2}$
- $\alpha = \mathrm{atan2}(dy, dx) - \theta$
- $\beta = -\theta - \alpha$

Las velocidades de control se calculan como:
- $v = K_{\rho} \cdot \rho$
- $w = K_{\alpha} \cdot \alpha + K_{\beta} \cdot \beta$

Donde $K_{\rho} > 0$, $K_{\alpha} > K_{\rho}$ y $K_{\beta} < 0$ son constantes de control.