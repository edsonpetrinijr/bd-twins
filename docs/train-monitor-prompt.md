# Prompt: treinar e acompanhar (sessão nova do Claude Code no PC de treino)

Cole tudo abaixo da linha numa sessão nova do Claude Code aberta na pasta `bd-twins`.

---

Você vai treinar e acompanhar a policy de andar do Atlas neste PC (Windows, GPU NVIDIA, sem admin). Repo: esta pasta (`bd-twins`), já com `.venv` instalado. Detalhes do projeto em `README.md` e `docs/atlas-sources.md`.

## 1. Preparar e disparar (uma vez)
1. `git pull` pra pegar a versão mais nova.
2. Confira a GPU: `.venv\Scripts\python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"`. Se der `False`, pare e me avise.
3. Dispare o treino em segundo plano, com o log indo pra arquivo (não prenda o terminal):
   ```powershell
   $env:WANDB_MODE = "disabled"
   Start-Process -FilePath ".venv\Scripts\python.exe" `
     -ArgumentList "atlas\train.py","train","Mjlab-Velocity-Flat-BD-Atlas","--env.scene.num-envs","4096","--agent.max-iterations","3000" `
     -RedirectStandardOutput "train.log" -RedirectStandardError "train.err" -NoNewWindow -PassThru
   ```
   Anote o PID. Se em 5 min aparecer "out of memory" no `train.err`, mate o processo e rode de novo com `2048`.
4. A pasta do run fica em `logs\rsl_rl\atlas_velocity\<data_hora>\` (checkpoints `model_*.pt` a cada 50 iterações).

## 2. Acompanhar a cada 30 min
Use `/loop 30m` (ou o agendamento que tiver disponível) com esta checagem:
- Últimas linhas do `train.log`: iteração atual de 3000, `Mean reward`, `Mean episode length`, erro de velocidade (`track_lin_vel` / `track_angular_velocity`), `Iteration time`.
- Compare com a checagem anterior: reward subindo? episódio crescendo?
- Processo ainda vivo (`Get-Process -Id <PID>`)? `train.err` com erro novo?
- Tempo restante = (3000 - iteração) x tempo por iteração.

Mande pra mim uma atualização curta em português, tom de conversa, 2 a 4 frases, sem caminhos de arquivo. Exemplo: "Iteração 900 de 3000, reward subiu de 12 pra 31 e ele já fica de pé o episódio todo. Faltam uns 70 min." Use o canal de aviso que esta sessão tiver (Jarvis/notificação push/mensagem); se não tiver nenhum, deixe a atualização no terminal.

## 3. Quando algo der errado
- **Processo morreu / erro no `train.err`**: me mande a última mensagem de erro (até 10 linhas). Se for queda isolada, retome do último checkpoint (veja as opções de resume em `.venv\Scripts\python atlas\train.py train Mjlab-Velocity-Flat-BD-Atlas --help`).
- **NaN na reward ou na loss**: pare e me avise; não recomece sozinho.
- **Reward parada por 300+ iterações antes de 1500**: me avise, mas deixe rodar.
- Não mude recompensas, gains nem o modelo do robô. Isso é decisão minha.

## 4. Quando terminar (3000 iterações ou reward estabilizada)
1. Me avise com os números finais.
2. Gere um vídeo do resultado se o `play` tiver opção de vídeo headless (confira o `--help` do `play`); senão me diga que dá pra abrir o viewer com `.venv\Scripts\python atlas\train.py play Mjlab-Velocity-Flat-BD-Atlas`.
3. Pare o `/loop`.
