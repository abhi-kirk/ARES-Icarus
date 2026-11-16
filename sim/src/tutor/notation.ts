/**
 * One notation sheet for Rocket Dynamics lessons, captions, and Ask.
 * Appended to every tutor context so chat, questions, and grades share names.
 */
export const NOTATION_SHEET = `Site notation (put every symbol in $...$):
- Height $x$ or $h$ (m). Rate $\\dot x$ or $v$ (m/s). Acceleration $\\ddot x$ or $a$.
- Scrubber time $t$ (s). Touchdown or final time $t_f$. Sample or pass index $k$.
- Mass $m$ (kg). Weight $W=mg$, $g=9.81\\,\\mathrm{m/s^2}$. Thrust $T$ or $F$ (N). Command $u$. Hover $F_0=mg$.
- Fuel $m_f$. Exhaust speed $v_e$. Error $e=x_{\\mathrm{ref}}-x$. Gains $K_p$, $K_i$, $K_d$.
- Open-loop growth $\\mu$ (1/s). Hidden mode $q$, growth $\\lambda$, coupling $\\varepsilon$.
- Pitch from vertical $\\theta$. Flight-path from horizontal $\\gamma$. Keep those two references apart.
- Attack $\\alpha$. Wind $W$. Dynamic pressure $q=\\tfrac12\\rho v^2$. Autopilot air coeff is $\\kappa$, never $q$.
- World: up is $+Y$; ground is the $X$–$Z$ plane.
- Trust radius $\\Delta_k$. Soft land on the missed scene needs $|\\dot z|\\le 0.5\\,\\mathrm{m/s}$ at the pad.
In prose use legend names (Open loop, Closed loop), never camelCase chart ids.`;
