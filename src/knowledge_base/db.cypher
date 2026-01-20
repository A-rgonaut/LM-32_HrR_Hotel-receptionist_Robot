MATCH (n) DETACH DELETE n;

CREATE

(o:Ospite {nome: "Peppe", cognome: "Rossi", eta: 30, lingua: "IT", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80}),
(o1:Ospite {nome: "Carletta", cognome: "Bianchi", eta: 40, lingua: "IT", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80}),
(o2:Ospite {nome: "Mr", cognome: "President", eta: 50, lingua: "EN", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80}),

(l1:Hall {x: 0.0, y: 10.0}),
(l2:CoffeeRoom {x: 0.0, y: -5.0}),
(l3:Stanza {x: 7.0, y: 4.2, nome_camera: "Camera 1"}),
(l4:Stanza {x: 7.0, y: -1.8, nome_camera: "Camera 2"}),
(l5:Stanza {x: 7.0, y: -7.8, nome_camera: "Camera 3"}),
(l6:Stanza {x: 2.5, y: -12.3, nome_camera: "Camera 4"}),
(l7:Stanza {x: -2.5, y: -12.3, nome_camera: "Camera 5"}),
(l8:Stanza {x: -7.0, y: 4.2, nome_camera: "Camera 6"}),
(l9:Stanza {x: -7.0, y: -1.8, nome_camera: "Camera 7"}),
(l10:Stanza {x: -7.0, y: -7.8, nome_camera: "Camera 8"}),

(p1:Prenotazione {data_inizio: datetime("2026-01-12T15:00:00"), data_fine: datetime("2026-01-16T12:00:00")}),

(o)-[:EFFETTUA]->(p1),
(p1)-[:ASSOCIATA_A]->(l3),

(r1:Robot {nome_robot: "Pippor"}),

(pat1:Cardiopatia {bpm_anomala: 100, bpm_allerta: 150, soglia_urgente: 1.5}),
(pat2:Ipertensione {pressione_min_allerta: 90, pressione_min_anomala: 100, pressione_max_allerta: 140, pressione_max_anomala: 160, soglia_urgente: 1.0}),

(o)-[:SOFFRE_DI]->(pat1),

(si1:Sintomo {nome_sintomo: "Palpitazioni", punteggio_base_sintomo: 0.5, punteggio_aggravante_sintomo: 1.5}),
(si2:Sintomo {nome_sintomo: "Vertigini", punteggio_base_sintomo: 0.3, punteggio_aggravante_sintomo: 1.2}),
(si3:Sintomo {nome_sintomo: "Pressione Toracica", punteggio_base_sintomo: 0.2, punteggio_aggravante_sintomo: 0.8}),

(si1)-[:CRITICA_PER]->(pat1),
(si2)-[:CRITICA_PER]->(pat2),
(si3)-[:CRITICA_PER]->(pat1),
(si3)-[:CRITICA_PER]->(pat2),

(i1:Interesse:Citta {nome_interesse: "vino"}),

(o)-[:HA_INTERESSE]->(i1),

(e1:EventoCitta {nome_evento_locale: "Degustazione Cantina Cellaro", data_ora_evento_locale: datetime("2026-01-14T21:00:00")}),
(e2:EventoMontagna {nome_evento_locale: "Escursione Monte Genuardo", data_ora_evento_locale: datetime("2026-01-14T08:00:00")}),
(e3:EventoMare {nome_evento_locale: "Giro in barca Porto Palo", data_ora_evento_locale: datetime("2026-01-14T12:00:00")}),

(s1:Specialista {nome: "Giuseppe", cognome: "Verdi", specialita: "elettrico", numero_telefono: "333 3333 333"}),
(s2:Specialista {nome: "Francesco", cognome: "Bianchi", specialita: "idraulico", numero_telefono: "333 4433 333"}),
(s3:Specialista {nome: "Giulio", cognome: "Neri", specialita: "medico", numero_telefono: "333 5533 333"}),

(m0:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T00:00:00"), condizione: "sereno", gradi: 10.0}),
(m1:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T01:00:00"), condizione: "sereno", gradi: 9.9}),
(m2:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T02:00:00"), condizione: "sereno", gradi: 9.9}),
(m3:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T03:00:00"), condizione: "sereno", gradi: 9.8}),
(m4:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T04:00:00"), condizione: "sereno", gradi: 9.7}),
(m5:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T05:00:00"), condizione: "sereno", gradi: 9.7}),
(m6:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T06:00:00"), condizione: "sereno", gradi: 10.2}),
(m7:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T07:00:00"), condizione: "sereno", gradi: 12.0}),
(m8:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T08:00:00"), condizione: "sereno", gradi: 13.0}),
(m9:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T09:00:00"), condizione: "nuvoloso", gradi: 11.0}),
(m10:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T10:00:00"), condizione: "nuvoloso", gradi: 10.8}),
(m11:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T11:00:00"), condizione: "pioggia", gradi: 10.0}),
(m12:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T12:00:00"), condizione: "pioggia", gradi: 10.0}),
(m13:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T13:00:00"), condizione: "pioggia", gradi: 9.9}),
(m14:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T14:00:00"), condizione: "pioggia", gradi: 9.7}),
(m15:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T15:00:00"), condizione: "sereno", gradi: 10.0}),
(m16:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T16:00:00"), condizione: "sereno", gradi: 10.1}),
(m17:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T17:00:00"), condizione: "sereno", gradi: 10.1}),
(m18:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T18:00:00"), condizione: "sereno", gradi: 10.0}),
(m19:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T19:00:00"), condizione: "sereno", gradi: 10.0}),
(m20:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T20:00:00"), condizione: "sereno", gradi: 10.0}),
(m21:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T21:00:00"), condizione: "sereno", gradi: 9.9}),
(m22:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T22:00:00"), condizione: "sereno", gradi: 9.9}),
(m23:PrevisioneMeteo {data_ora_meteo: datetime("2026-01-14T23:00:00"), condizione: "sereno", gradi: 10.0}),

(a1:Anomalia {
    nome_anomalia: "Puzza di bruciato",
    punteggio_base_anomalia: 0.5,
    punteggio_aggravante_anomalia: 2.0
}),
(a2:Anomalia {
    nome_anomalia: "Fa rumore forte",
    punteggio_base_anomalia: 0.1,
    punteggio_aggravante_anomalia: 0.9
}),
(a3:Anomalia {
    nome_anomalia: "Perde acqua",
    punteggio_base_anomalia: 0.3,
    punteggio_aggravante_anomalia: 1.2
}),
(a4:Anomalia {
    nome_anomalia: "Non emette aria fredda",
    punteggio_base_anomalia: 0.1,
    punteggio_aggravante_anomalia: 1.9
}),

(o)-[:AVVERTE]->(a1),
(o)-[:AVVERTE]->(a2),
(o)-[:AVVERTE]->(a3),
(o)-[:AVVERTE]->(a4),

(og1:Condizionatore {
    tipo_oggetto: "elettrico",
    tempo_acceso: 35,
    temperatura_stanza: 18,
    temperatura_impostata: 22,
    modalita: "HEAT",
    soglia_assistenza: 2.0
}),
(og2:Lavandino {
    tipo_oggetto: "idraulico",
    soglia_assistenza: 1.0
}),
(og3:Phon {
    tipo_oggetto: "elettrico",
    soglia_assistenza: 2.0
}),

(a1)-[:DANNOSO_PER]->(og3),
(a2)-[:DANNOSO_PER]->(og1),
(a2)-[:DANNOSO_PER]->(og2),
(a2)-[:DANNOSO_PER]->(og3),
(a3)-[:DANNOSO_PER]->(og2),
(a4)-[:DANNOSO_PER]->(og1),

(g1:Guasto {
    data_ora_guasto: datetime(),
    assistenza: true
}),
(g2:Guasto {
    data_ora_guasto: datetime(),
    assistenza: true
}),
(g3:Guasto {
    data_ora_guasto: datetime(),
    assistenza: true
}),

(o)-[:SEGNALA]->(g1),
(o)-[:SEGNALA]->(g2),
(o)-[:SEGNALA]->(g3),

(g1)-[:INERENTE_A]->(og1),
(g2)-[:INERENTE_A]->(og2),
(g3)-[:INERENTE_A]->(og3),

(r1)-[:NOTIFICA]->(s1),
(r1)-[:NOTIFICA]->(s2),
(r1)-[:NOTIFICA]->(s1)
