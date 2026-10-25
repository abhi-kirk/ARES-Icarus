/** Muted mark for the empty Ask thread. */
export function RocketMark() {
  return (
    <svg className="chat-mark" viewBox="0 0 120 200" aria-hidden="true">
      <g transform="rotate(-18 60 100)">
        <rect x="52" y="40" width="16" height="100" rx="2" fill="currentColor" opacity="0.35" />
        <polygon points="60,18 72,42 48,42" fill="currentColor" opacity="0.45" />
        <polygon points="52,140 60,168 68,140" fill="currentColor" opacity="0.3" />
        <line x1="48" y1="130" x2="36" y2="150" stroke="currentColor" strokeWidth="2" opacity="0.35" />
        <line x1="72" y1="130" x2="84" y2="150" stroke="currentColor" strokeWidth="2" opacity="0.35" />
      </g>
    </svg>
  );
}
