Agisci come un esperto di sistemi diagnostici e comunicazione tecnica. 
Analizza la seguente regola logica formale:
Stufa(?s) ^ temperatura_impostata(?s, ?tem_i) ^ tempo_acceso(?s, ?tem_acc) ^ temperatura_stanza(?s, ?tem_sta) ^ tempo_acceso_soglia(?s, ?tem_sogl) ^ swrlb:greaterThan(?tem_i, ?tem_sta) ^ swrlb:greaterThan(?tem_acc, ?tem_sogl) -> GuastoElettrico(?s)

OBIETTIVO:
Genera una spiegazione sintetica (massimo 20 parole) da fornire a un utente finale (ospite) per giustificare l'intervento di uno specialista. 

REQUISITI RIGOROSI:
1. Generalizzazione: Non riferirti necessariamente a una "stufa", ma al malfunzionamento del componente in relazione al suo obiettivo.
2. Causalità: La spiegazione deve riflettere il fatto che il dispositivo è rimasto attivo oltre il tempo limite senza produrre l'effetto richiesto.
3. Tono: Formale, univoco, privo di espressioni colloquiali o superflue.
4. Output: Restituisci solo il testo della spiegazione, completando la frase "Ti chiamo lo specialista perché...".