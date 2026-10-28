import { useState } from "react";
import { COMING_SOON, DOORS } from "./doors";
import { follow } from "./route";

export function TopBar({
  dynamics,
  current,
  asking,
  onAsk,
}: {
  dynamics: boolean;
  current: string;
  asking: boolean;
  onAsk: () => void;
}) {
  const [note, setNote] = useState("");

  return (
    <header className="topbar">
      <a className="wordmark" href="/" onClick={(event) => follow(event, "/")}>
        Icarus
      </a>
      <div className="nav-block">
        <nav className="nav" aria-label="Site">
          {DOORS.map((door) =>
            door.href ? (
              <a
                key={door.id}
                className={door.id === current ? "nav-link is-current" : "nav-link"}
                href={door.href}
                onClick={(event) => {
                  setNote("");
                  follow(event, door.href);
                }}
              >
                {door.name}
              </a>
            ) : (
              <button key={door.id} className="nav-link" type="button" onClick={() => setNote(COMING_SOON)}>
                {door.name}
              </button>
            ),
          )}
          {note ? <span className="nav-note">{note}</span> : null}
        </nav>
      </div>
      <div className="bar-end">
        {dynamics ? (
          <button type="button" className="ask" data-ask-toggle aria-pressed={asking} onClick={onAsk}>
            {asking ? "Close" : "Ask"}
          </button>
        ) : null}
        <a className="contact" href="mailto:abhikirk@icloud.com">
          Abhi Jain
        </a>
      </div>
    </header>
  );
}
