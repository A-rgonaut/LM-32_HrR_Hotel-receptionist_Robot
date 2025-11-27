# Scenario C – “Rilevazione di un malore improvviso: Franco in Caffè-Room”

**Contesto generale:**

È un tardo pomeriggio di marzo. Il signor Franco (45 anni, di nazionalità italiana, con una nota condizione cardiaca pregressa), si trova da solo nella Caffè-Room dell’hotel (adiacente alla reception) dopo una sessione lavorativa particolarmente stressante. L'ospite indossa un braccialetto smart fornito dall’hotel che monitora in tempo reale i suoi parametri vitali.

**Attori coinvolti:**

- *Franco*: Ospite della struttura alberghiera.
- *Pippor (HrR - Hotel-receptionist Robot)*: Un robot posizionato nella hall dell’hotel, dotato di avanzate capacità conversazionali e di monitoraggio della salute dell’ospite. Fornisce un primo livello di supporto e interazione per richieste generiche e situazioni di emergenza/malessere.
- *Sistema di monitoraggio (integrato)*: La rete di sensori (braccialetto e sistema centrale) che raccoglie e analizza i dati vitali.

## Fase 1: Rilevazione del contesto e allerta (monitoraggio proattivo)
Il sistema di monitoraggio riceve e analizza il flusso di dati dal braccialetto di Franco.

- <u>Anomalia iniziale</u>: Pippor rileva che la frequenza cardiaca di Franco ha raggiunto i 150 battiti al minuto (bpm), un valore significativamente elevato rispetto al suo baseline e alla sua condizione.

- <u>Verifica della persistenza</u>: Pippor, riconoscendo che potrebbe trattarsi di una variazione momentanea (es. un colpo di tosse o un movimento brusco), attiva un protocollo di verifica: ripete l'osservazione e l'analisi del dato dopo un intervallo di pochi minuti.

- <u>Attivazione dell'allerta</u>: Constatando che la condizione (frequenza cardiaca elevata) continua a perdurare e non mostra segni di normalizzazione autonoma, Pippor innesca il piano d'azione di supporto e intervento.

## Fase 2: Pianificazione del percorso e intervento fisico
Pippor deve raggiungere rapidamente l'ospite per una valutazione sul campo.

- <u>Localizzazione e pianificazione</u>: Utilizzando la posizione GPS rilevata dal braccialetto di Franco e la propria conoscenza della mappa dell'albergo (incluse le posizioni note di ostacoli statici, come mobili o pilastri), Pippor calcola il percorso ottimale per raggiungere la Caffè-Room.

- <u>Navigazione dinamica</u>: Durante lo spostamento, Pippor utilizza i propri sensori di navigazione per verificare in tempo reale la presenza di ostacoli dinamici (altri ospiti, carrelli, ecc.) e aggiornare il percorso se necessario.

- <u>Ingresso in stanza</u>: Pippor completa con successo il percorso e si posiziona in prossimità di Franco.
 
## Fase 3: Valutazione della gravità e supporto dialogico
Pippor avvia l'interazione per comprendere lo stato di coscienza e la gravità dei sintomi.

- <u>Stato dell'Utente</u>: Franco è sdraiato sul divano, visibilmente agitato, ma mantiene lo stato di coscienza. La frequenza cardiaca è scesa a 130 bpm.

- <u>Avvio del dialogo</u>: Pippor stabilisce un contatto vocale, mantenendo un tono di voce calmo e rassicurante:

    *Pippor*: "`Signor Franco, buonasera. Dalla rilevazione in tempo reale inviatami dal Suo braccialetto, mi risulta che Lei sia in una situazione di malessere. Può confermarmi che mi sente e che è cosciente, per favore?`"

- <u>Descrizione dei sintomi</u>: Franco risponde affermativamente. Pippor prosegue:
    
    *Pippor*: "`Grazie, Franco. Potrebbe descrivermi i Suoi sintomi attuali? Se lo desidera, mi dica cosa prova.`"

    *Franco*: "Ho un forte giramento di testa e sto sudando, ma per fortuna non mi tremano le mani come a volte succede."

- <u>Analisi dei dati</u>: Pippor elabora i sintomi descritti (sudorazione, vertigini) in relazione ai dati vitali in calo (130 bpm), classificando la situazione come non grave e non immediatamente pericolosa per la vita.

## Fase 4: Normalizzazione e intervento autonomo rassicurante
Pippor continua l'assistenza, monitorando i parametri in fase di recupero.

- <u>Rientro nei parametri</u>: La frequenza cardiaca di Franco scende ulteriormente a 110 bpm, un valore considerato regolare e stabile per un soggetto con la sua condizione cardiaca.

- <u>Rinforzo positivo</u>: Pippor fornisce supporto psicologico e conferma la normalizzazione in corso, agendo in modo autonomo e rassicurante:

    *Pippor*: "`Ottimo Franco. I suoi parametri si stanno già normalizzando. La frequenza cardiaca è ora stabile a 110 battiti. È stata una situazione temporanea. Non è necessario richiedere l'intervento del personale medico in questo momento. Resto qui con Lei.`"

- <u>Conclusione</u>: Franco sorride in segno di sollievo e gratitudine. Pippor continua il monitoraggio sul posto finché la frequenza cardiaca non scende sotto la soglia di allerta stabilita.

