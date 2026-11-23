# Pod battery: diagnosi CRC — 21/11/2026

Diagnosi e correzione assistite da Codex dopo gli ID batteria incoerenti osservati durante l'installazione.
Non sono stati comandati motori, switch o armamento sul ROV, né modificati firmware o parametri del veicolo.

## Difetto del ricevitore verificato

La precedente premessa «il parser C passa a Lua soltanto pacchetti con CRC valido» era sbagliata.
`libraries/GCS_MAVLink/GCS_Common.cpp`, `GCS_MAVLINK::update_receive`, inoltra esplicitamente
`MAVLINK_FRAMING_BAD_CRC` a scripting per permettere l'uso di dialect non conosciuti dal firmware.
Il nostro applet usa messaggi conosciuti ma aveva omesso la verifica CRC per ridurre il costo di decodifica.

Una riproduzione in SITL ha inviato BATTERY_STATUS con sysid/compid/id corretti, tensione 11,111 V
e CRC deliberatamente alterato. Prima della correzione il dato sostituiva la tensione valida e
il traffico corrotto impediva il normale timeout. **40/43 controlli passavano**, con tre fallimenti CRC.

Ora il CRC viene confrontato con header e payload effettivamente trasmessi, **prima** del riempimento
degli zeri finali e della decodifica. Un pacchetto rifiutato non aggiorna misura, freschezza o stato di alimentazione.
Il limite per chiamata è tre messaggi dal mittente accettato, mantenendo 40 Hz e la coda di 25 elementi.
Il filtro sysid/compid resta prima del calcolo CRC.

`PBCR` registra `Comp,Crc` ogni secondo. `PBAD` registra i metadati dei primi tre rifiuti per pod
per avvio script. `PBAT` conserva esattamente il formato originario: un nuovo campo nello stesso
nome di log non è compatibile con un semplice riavvio dello scripting nello stesso processo autopilota.
Il CRC verifica integrità, non autenticità del mittente.

## Catture passive sul Pi

- Ethernet, filtro UDP 14560, circa 35 s: **1044 frame emessi dai pod**, tutti con CRC valido
  nelle definizioni note. BATTERY_STATUS: 348 per pod; nessun ID batteria errato osservato in questa cattura.
- Loopback, filtro UDP 8852, circa 25 s: **734 frame pod inoltrati all'autopilota**, tutti con CRC valido.
  BATTERY_STATUS: 245 per pod.
- BlueOS invia anche molti messaggi ai client del proprio UDP Server: nella cattura Ethernet
  6747/6745 datagrammi sono tornati dal Pi ai pod sulla porta 14591. Finivano nel parser ASCII CSV
  dei pod e potevano rendere il JSON di debug non UTF-8 nel campo `last_rx`.
  La correzione sul pod è inclusa nel firmware 0.8.10, senza flash durante questa attività.

Queste finestre **non dimostrano** dove nasca ogni pacchetto consegnato a Lua come BAD_CRC.
Nella prima osservazione con il controllo CRC attivo, il DataFlash `PBAD` ha effettivamente registrato
pacchetti con ID non plausibili (es. 63 e 255) e CRC rifiutato; `PBAT.Ign` è rimasto zero mentre
le batterie continuavano sane. Questo distingue il problema da uno scambio fisso BAT1/BAT2.
Resta da localizzare l'origine degli errori intermittenti nel percorso di ricezione, senza attribuirli
agli ESP32 sulla sola base di questa evidenza.

## Verifica e installazione sul Pi

- SITL funzionale aggiornato: **43/43**, inclusi rifiuto della tensione corrotta e mancato rinnovo del timeout.
- Prova carico con frequenza realmente misurata: **13/13**, circa 29 Hz BATTERY_STATUS per pod
  contro 30 Hz richiesti, circa 121 messaggi/s estranei; 938/938 BATTERY_STATUS e 376/376 named-value
  ricevuti per pod, nessuna falsa perdita di salute o errore VM osservato.
- Un precedente tentativo di carico non aveva raggiunto la frequenza nominale: non viene usato come
  prova di capacità. Il test ora controlla anche il rate raggiunto e usa un clock monotono.
- Python flake8 e `git diff --check` superati.

Installato il solo `pod_battery.lua`, riavviando solo scripting con ROV disarmato;
ACK del comando accettato e `PodBatt: started` osservato. Nessun nuovo parametro impostato.
I tre moduli MAVLink restano invariati.
Backup del precedente script:
`/root/.config/blueos/ardupilot-manager/pod-battery-backups/pod_battery-before-crc-20261121.lua`.
SHA-256 del file definitivo, confrontato localmente e sul Pi:
`858b0e05f223042f0a08112cf13c0f309cf7ae22d820e31449d21a54419cc1a2`.

Nella verifica finale di 20 s, entrambe le batterie sono ripubblicate sane da ArduSub,
con 200 frame pod e 30 frame autopilota per batteria. Snapshot: BAT1 45,512 V / 0,34 A,
BAT2 45,787 V / 0,14 A; percentuale residua -1. Sono valori di telemetria, non tarature.
È stato necessario ripetere una richiesta di riavvio senza ACK, usando un'identità GCS distinta;
la richiesta finale ha ricevuto ACK. Non è stato riavviato ArduSub o il Pi.

## Evidenze

Pacchetto locale: `/Users/deepex/Documents/Codex/2026-11-21/vbus-web-and-mavlink-diagnosis/`.
Include PCAP, analisi CRC, test prima/dopo, verifica di carico, catture MAVLink e riepiloghi DataFlash.
Le finestre DataFlash ottenute concatenando l'inizio con la coda del file sono snapshot parziali,
non log di volo continui: le discontinuità di estrazione non sono diagnostica di guasto del logger.
Cockpit, interruzioni fisiche dei pod e comportamento in acqua restano fuori da questa verifica.
