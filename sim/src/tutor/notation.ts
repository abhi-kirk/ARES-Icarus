/**
 * One notation sheet for Rocket Dynamics lessons, captions, and Ask.
 * Injected into every tutor context so chat, questions, and grades share the same names.
 */
export const NOTATION_SHEET = `Site notation (use these names and symbols; put every symbol in $...$):
- Height / altitude from the pad or rest point: $x$ or $h$ (metres). Rate: $\\dot x$ or $v$ (m/s). Acceleration: $\\ddot x$ or $a$ (m/s²).
- Time on the scrubber: $t$ (s). Final or touchdown time: $t_f$ (s). Sample index: $k$. Pass index in successive methods: $k$.
- Mass $m$ (kg). Weight $W = mg$ with $g = 9.81\\,\\mathrm{m/s^2}$. Thrust $T$ or $F$ (N). Command $u$. Hover thrust $F_0 = mg$.
- Fuel left $m_f$ (kg). Exhaust speed $v_e$ (m/s). Specific impulse ideas use $v_e$, not a second symbol for the same thing.
- Error $e = x_{\\mathrm{ref}} - x$. Gains $K_p$, $K_i$, $K_d$. Open-loop growth $\\mu$ (1/s). Hidden mode $q$, growth $\\lambda$, coupling $\\varepsilon$.
- Pitch from vertical: $\\theta$ (degrees on knobs, radians in formulas unless stated). Flight-path angle from the horizontal: $\\gamma$. Do not mix the two references in one answer.
- Angle of attack $\\alpha$. Wind speed $W$. Dynamic pressure ideas use $q = \\tfrac12\\rho v^2$; the air restoring coefficient on the autopilot scene is $\\kappa$, never $q$.
- World frame on this site: height is up along $+Y$; the ground plane is $X$–$Z$. Body nose lean uses that same vertical.
- Trust radius $\\Delta_k$. Soft landing needs $|\\dot z| \\le 0.5\\,\\mathrm{m/s}$ at the pad on the missed-landing scene.
Never write camelCase chart ids (openHeight) in prose. Prefer the legend names Open loop / Closed loop.`;

export function notationBlock(): string {
  return NOTATION_SHEET;
}
