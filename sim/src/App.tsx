import { useEffect, useState } from "react";
import { Landing } from "./Landing";
import { DynamicsModel } from "./pages/DynamicsModel";
import { CoastLook } from "./pages/CoastLook";
import { SimulatorPage } from "./pages/Simulator";
import { TopBar } from "./TopBar";
import { isDynamicsPath, redirectForPath, sceneFromPath } from "./paths";
import { usePath, visit } from "./route";
import { setScene } from "./sceneStore";

export function App() {
  const path = usePath();
  const coast = path === "/simulator/coast";
  const simulator = path === "/simulator" || coast;
  const dynamics = !simulator && (isDynamicsPath(path) || redirectForPath(path) !== null);
  const [asking, setAsking] = useState(false);

  useEffect(() => {
    const redirect = redirectForPath(path);
    if (redirect) {
      visit(redirect);
      return;
    }
    const sceneId = sceneFromPath(path);
    if (sceneId) setScene(sceneId);
  }, [path]);

  useEffect(() => {
    document.title = simulator ? "Icarus · Simulator" : dynamics ? "Icarus · Rocket Dynamics" : "Icarus";
  }, [dynamics, simulator]);

  const stage = dynamics || simulator;

  return (
    <div className={stage ? "site" : "site is-cover"}>
      <TopBar
        dynamics={dynamics}
        current={simulator ? "simulator" : dynamics ? "dynamics" : ""}
        asking={asking}
        onAsk={() => setAsking((open) => !open)}
      />
      {coast ? <CoastLook /> : simulator ? <SimulatorPage /> : dynamics ? <DynamicsModel open={asking} onClose={() => setAsking(false)} /> : <Landing />}
    </div>
  );
}
