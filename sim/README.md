# Icarus-S

The public side of Icarus, at [icarus-arc.com](https://icarus-arc.com). One argument, told in scenes: how a rocket comes down.

A scene is a picture you can move and a chart drawn from the same samples. The page holds the numbers. Ask opens over the picture. The writing stays put. A reply can move the time, or ask for a series that is already in the buffer. It does not invent rows.

## Rocket Dynamics

Four scenes, one clock.

The spring. A mass hangs from a spring with the damper open. Height from the rest point is the state, and the rate is how fast that height is changing. At the top of the swing, and again at the bottom, the mass turns around and the rate is zero.

The step. The spring force sets the rate, and each step adds that rate times the time step before holding the new height. The wire box is the true height and the gold mass is the step. A coarse step feeds energy in, so the gold mass swings wider.

The burn. Thrust stays constant while the tank empties, so the acceleration climbs, and at four seconds the thrust stops.

The engine. The command jumps and the engine follows, but the thrust cannot pass 50 N.

Under the title, that column scrolls: the equations, then an Icarus Q&A card with beginner, intermediate, and advanced questions. A typed answer is graded by Grok. Ask can move the time or plot a series. When the reader asks for help with a card question, Ask explains it and says whether an answer they offer holds.

Play runs that clock in real time. The live chart draws only as far as the picture has gone. A chart the conversation asks for draws the whole run, with the same time marked.

The home page is the name, the two doors, and a vertical takeoff and landing beside them. The booster leaves the top of the screen and comes back to the pad. Both doors open. Rocket Dynamics continues from this page through the loop, the frequency, the turn, the vehicle, the best descent, and the propellant.

The questions and Ask call the Worker. The Grok key is the secret `GROK_API_KEY`. The page takes a one-hour cookie from `GET /api/session`, signed with `ICARUS_SESSION_SECRET`, and `/api/tutor` and `/api/chat` refuse a call that does not carry it. There is no login.

## Simulator

Earth opens on the climb that starts when the booster leaves: 100 km, 2.0 km/s, flight path 23.5°, with 5,000 kg of fuel. The booster starts stacked on the tail and slides straight aft over the first 12 seconds. The plume is the same jet as the landing. The engine cuts off near 200 km and 7,700 m/s. Return is the fall from 120 km at 7.6 km/s. The windward side takes the air until 2 km/s, then the nose swings onto the velocity, the engines light, and the hop laws finish the landing. Landing is that hop on its own. Drop starts over the pad. Divert starts 70 m downrange and 40 m to the side. The Moon is that divert, with no air and a pad that stays put. PID, MPC, and SCvx fly the landing. The climb and the return play recorded flights, because a request from the page is still clamped into a hop.

The left panel is the case. The right panel is the plots. On the climb and the return the top plot is altitude in kilometers, and the return also draws the heat rate, sqrt(rho) times speed cubed. The landing plots stay in meters: altitude, range, thrust, and vertical speed. Drag the picture to look around. Scroll zooms. Command-drag, Shift-drag, and the right button pan.

## Run

From this folder, `npm run dev` serves http://127.0.0.1:5174/. Copy `.dev.vars.example` to `.dev.vars` and fill in the two secrets, or Ask and the questions will not answer. A push to `main` deploys the site. GitHub Actions builds this folder and runs Wrangler against the config the build writes under `dist/`.
