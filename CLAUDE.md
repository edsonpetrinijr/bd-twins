# bd-twins

- Modelos MJCF de robôs da Boston Dynamics feitos **só de dados públicos**. Todo número novo entra em `docs/<robô>-sources.md` com fonte ou marcado como estimativa.
- Modelado do zero: não copiar estrutura de outro robô (G1 etc.). Do mjlab, só as convenções que ele exige (sem atuador no XML, `floating_base_joint`, sites `imu_in_*`, `*_foot`, `*_palm`).
- Rodar `.venv/bin/python atlas/check.py` depois de qualquer mudança no XML.
- Passos pequenos, 1 commit por passo.
- Estado em `STATE.md`.
