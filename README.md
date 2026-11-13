See: https://www.ittc.ku.edu/~heechul/papers/rme-adams2025-camera.pdf

## Per-config pipelines

The trapper, return control unit, and one column extractor are shared. Each
config owns its AGU, requestor, prefetch unit, fetch request table, DRAM/LLC
response buffers, and packer bank. The extractor registers the destination
packer with each accepted fragment and can emit the previous fragment while
accepting the next, sustaining one fragment per cycle when destinations are
ready. Admission checks reserve packer capacity before accepting a fragment;
an extractor output stall backpressures all configs. Packer reservations are
released only when the completed line is accepted by the trapper.

Set packer counts at elaboration through `RelMemParams`:

```scala
RelMemParams(maxConfigs = 4, nPackersPerConfig = Seq(2, 4, 1, 3))
```

An empty `nPackersPerConfig` (the default) provisions two packers per config.
Otherwise provide exactly one positive count per config. Banks do not borrow
packers from each other. These counts do not change the software register ABI.

`nFetchUnits` remains the **total** number of outstanding fetch-table entries,
partitioned across configs. Any remainder goes to the lowest config indices;
there must be at least one entry per config. The default 16 entries and four
configs therefore provide four entries per pipeline. Source IDs encode both
pipeline and local entry on the shared DRAM and LLC ports.

Directory probes use one shared, pipelined port accepting up to one request per
cycle. The existing directory's one-cycle response is returned only to its
requesting pipeline while the next probe can be accepted. Memory
ports, the control unit's single fetched-line input, completed-line arbitration,
and trapper reply serialization remain shared. `ColumnWidths` and the other
existing global format registers remain global.

Focused RTL regressions are in the enclosing Chipyard checkout's `tests/dtu/`.
