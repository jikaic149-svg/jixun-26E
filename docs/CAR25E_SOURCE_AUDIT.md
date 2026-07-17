# Car25E source audit

Reference repository:

- URL: `https://github.com/jikaic149-svg/-25E`
- branch: `main`
- audited commit: `c181f1c74b5a2bf29f1e3abd9230f381c00a01be`

The repository has no root LICENSE. Therefore the template does not assume
that every file can be copied or relicensed merely because the repository is
public or owned by the user account.

The following 14 files explicitly state `@origin 云台` and require separate
authorization or an independent rewrite before entering a distributable
template:

```text
APP/algorithm/crc16.c
APP/algorithm/crc16.h
APP/algorithm/crc8.c
APP/algorithm/crc8.h
APP/algorithm/kalman_filter.c
APP/algorithm/kalman_filter.h
APP/algorithm/math_utils.c
APP/algorithm/math_utils.h
APP/algorithm/quaternion_ekf.c
APP/algorithm/quaternion_ekf.h
APP/framework/daemon.c
APP/framework/daemon.h
APP/framework/message_center.c
APP/framework/message_center.h
```

The current template copies none of these files. `Mode/Car`, `Mode/Gimbal`,
`Mode/Car/Control`, `Task`, the reusable APP/Algorithm/Device modules, and
`Task/Hex25E` are new implementations of confirmed functional requirements;
they are not source copies or compatibility wrappers. The template does not
copy the older FreeRTOS tree, generated SysConfig output, Keil objects/logs,
Python cache files, or PZ100 pin assignments.

`Task/Hex25E` uses only these factual inputs from audited commit
`c181f1c74b5a2bf29f1e3abd9230f381c00a01be`: the 14 node names, their graph
adjacency/headings, the five degree-3 junctions, the nominal 400 mm regular
hexagon side, the 210 mm O-to-first-Y launch spur, encoder conversion values,
and the requested observable behavior.
The traversal state machine, shortest-route derivation, safety gates, public
API, and integration with this template were independently written. An
exhaustive `(node, heading, visited-Y-mask)` breadth-first search establishes
that the selected closed route requires 12 edges under the map's +/-60 degree
turn constraint.

Before importing any other Car25E source, confirm its ownership/provenance and
record the result in the module `README.md` and, for third-party material, in
the root `THIRD_PARTY_NOTICES.md`. Functional behavior may instead be
reimplemented from public specifications under the policy in
`INDEPENDENT_REWRITE_POLICY.md`.
