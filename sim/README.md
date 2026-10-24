# Icarus-S

The public side of Icarus, at [icarus-arc.com](https://icarus-arc.com). One argument, told in scenes: how a rocket comes down.

A scene is a picture you can move and a chart drawn from the same samples. The page holds the numbers. Ask opens over the picture. The writing stays put. A reply can move the time, or ask for a series that is already in the buffer. It does not invent rows.

## Rocket Dynamics

Four scenes, one clock.

A mass on a spring, with the damper open. Height from the rest point is the state. The rate is how fast that height is changing. At the top, and again at the bottom, the mass turns around and the rate is zero.

A numerical step. The spring force sets the rate. Each step adds that rate times a time step and holds it. The wire box is the true height. The gold mass is the step. A coarse step feeds energy in, so the gold mass swings wider.

A mass that changes. Thrust stays constant while the tank empties, so the acceleration climbs. At four seconds the thrust stops.

An engine with a lag and a limit. The command jumps. The engine follows, and it cannot pass 50 N.

Under the title, that column scrolls: the equations, then an Icarus Q&A card with beginner, intermediate, and advanced questions. A typed answer is graded by Grok. Ask can move the time or plot a series. It does not answer a question on a card.

Play runs that clock in real time. The live chart draws only as far as the picture has gone. A chart the conversation asks for draws the whole run, with the same time marked.

The home page is the name, the two doors, and a vertical takeoff and landing beside them. The booster leaves the top of the screen and comes back to the pad. Simulator says Coming Soon. The chapters after this page, the loop, the turn, the vehicle, and the descent, stay out of the nav until they are drawn.

The questions and Ask call the Worker. The Grok key is the secret `GROK_API_KEY`. The page takes a one-hour cookie from `GET /api/session`, signed with `ICARUS_SESSION_SECRET`, and `/api/tutor` and `/api/chat` refuse a call that does not carry it. There is no login.

## Run

From this folder, `npm run dev` serves http://127.0.0.1:5174/. Copy `.dev.vars.example` to `.dev.vars` and fill in the two secrets, or Ask and the questions will not answer. Deploy with `npm run build && npx wrangler deploy`. Wrangler publishes the config the build writes under `dist/`.
