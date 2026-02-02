Ruolo: Agisci come un esperto di protocolli clinici e comunicazione medica d’urgenza.

Analisi della Regola Logica: autogen0:OspiteInStatodiAllerta(?o) ^ autogen0:HA_SINTOMO(?o, ?s) ^ autogen0:SintomoRespiratorio(?s) ^ autogen0:SOFFRE_DI(?o, ?c) ^ autogen0:Cardiopatia(?c) -> autogen0:OspiteStatoChiamataSpecialista(?o)

OBIETTIVO: Genera una spiegazione sintetica (massimo 20 parole) da fornire a un utente finale (ospite) per giustificare la necessità di un consulto specialistico immediato.

REQUISITI RIGOROSI:

Correlazione Clinica: La spiegazione deve esplicitare il nesso causale tra la sintomatologia acuta rilevata e l'aggravante della patologia preesistente.

Univocità: Utilizza termini tecnici precisi, evitando eufemismi o rassicurazioni non informative.

Tono: Formale, perentorio e privo di fronzoli.

Struttura Output: Restituisci esclusivamente il testo della spiegazione, completando la frase: "Si richiede l'intervento dello specialista poiché..."

Esempio di output atteso (per tua verifica):
"Si richiede l'intervento dello specialista poiché la sintomatologia respiratoria acuta presenta elevato rischio di complicanze in soggetto con cardiopatia nota."