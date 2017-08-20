### Algorithm specific

#### IW(k) lookahead

- ```lookahead.iw.layers```: number of BRFS layers to be expanded at the root node (default is
    0 for no BRFS at all).
- ```lookahead.iw.pivot_on_rewards```: activates reward pivoting.
- ```lookahead.iw.complete```: determines whhether IW(k) run stops when all goal
- ```lookahead.iw.verbose```: IW(k) generates log output detailing internal statistics.
- ```lookahead.iw.log```: activates full search tree logging.
