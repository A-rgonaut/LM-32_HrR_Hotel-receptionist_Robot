MATCH (n) DETACH DELETE n;

CREATE

(o:Ospite {nome: "Peppe", cognome: "Rossi", eta: 30, lingua: "IT", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80, soglia_bpm_anomala: 100, soglia_bpm_allerta: 150}),
(o1:Ospite {nome: "Carletta", cognome: "Bianchi", eta: 40, lingua: "IT", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80, soglia_bpm_anomala: 120, soglia_bpm_allerta: 170}),
(o2:Ospite {nome: "Mr", cognome: "President", eta: 50, lingua: "EN", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80, soglia_bpm_anomala: 110, soglia_bpm_allerta: 160}),

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
(l11:Ricarica {x: 10.0, y: 11.0}),

(p1:Prenotazione {data_inizio: datetime("2026-01-12T15:00:00"), data_fine: datetime("2026-01-16T12:00:00")}),

(o)-[:EFFETTUA]->(p1),
(p1)-[:ASSOCIATA_A]->(l3),

(r1:Robot {nome_robot: "Pippor"}),

(pat1:Cardiopatia),
(pat2:Ipertensione),

(o)-[:SOFFRE_DI]->(pat1),

(i1:Citta {nome_interesse: "vino"}),
(i2:Montagna {nome_interesse: "trekking"}),

(o)-[:HA_INTERESSE]->(i1),
(o)-[:HA_INTERESSE]->(i2),

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

(og1:Condizionatore {
    tipo_oggetto_guastabile: "elettrico",
    tempo_acceso: 35,
    temperatura_stanza: 18,
    temperatura_impostata: 22,
    modalita: "HEAT"
}),
(og2:Lavandino {
    tipo_oggetto_guastabile: "idraulico"
}),
(og3:Phon {
    tipo_oggetto_guastabile: "elettrico"
}),

(g1:Guasto {
    tipo_oggetto_guasto: "elettrico"
}),
(g2:Guasto {
    tipo_oggetto_guasto: "idraulico"
}),
(g3:Guasto {
    tipo_oggetto_guasto: "elettrico"
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
