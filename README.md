# Icarus

Icarus is a powered-descent project in two parts that share one story.

**Icarus-S** (`sim/`) is the public site. It tells how a rocket comes down, with a picture you can move, a chart of the numbers, and a conversation beside them.

**Icarus-I** (`rockets/icarus-i/`) is the first airframe, a LOC IV being built for an L1 flight. Later rockets are Icarus-II and so on. A second flight of the same rocket is not a new number.

**engine/** is the earlier C++ landing, Phases 1 to 3. It still builds. The site does not call it. New scenes run in the browser so the picture and the chart share one buffer.

```bash
cd sim && npm install && npm run dev
cd engine && make run
```

What you do by hand for the domain and the Grok key is in `SETUP.md`.
