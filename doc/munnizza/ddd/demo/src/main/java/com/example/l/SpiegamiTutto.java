package com.example.l;

import java.io.File;
import java.util.Set;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;
import org.semanticweb.owlapi.apibinding.OWLManager;
import openllet.owlapi.OpenlletReasonerFactory; 
import com.clarkparsia.owlapi.explanation.DefaultExplanationGenerator;

public class SpiegamiTutto {
    public static void main(String[] args) {

        OWLOntologyManager man = OWLManager.createOWLOntologyManager();


        System.out.println("--- AVVIO SISTEMA DI RAGIONAMENTO (WINDOWS) ---");

        File fileOntologia = new File("C:/Users/paolo/Desktop/ddd/demo/src/main/java/com/example/l/ontologiaAlbergo.rdf");

        
        if (fileOntologia.exists()) {
            System.out.println("✅ Il file " + fileOntologia.getName() + " è stato trovato nel percorso: " + fileOntologia.getAbsolutePath());
            try {
                // Carica l'ontologia dal file
                OWLOntology ontologia = man.loadOntologyFromOntologyDocument(fileOntologia);
                OWLDataFactory df = man.getOWLDataFactory();
                System.out.println("Ontologia caricata con successo!");
                System.out.println("-> IRI/Nome dell'Ontologia: " + ontologia.getOntologyID().getOntologyIRI().orElse(IRI.create("Nessun nome esplicito")));
                System.out.println("-> Numero totale di Assiomi: " + ontologia.getAxiomCount());
                System.out.println("-> Numero di Classi: " + ontologia.getClassesInSignature().size());
                System.out.println("-> Numero di Data Properties: " + ontologia.getDataPropertiesInSignature().size());
                System.out.println("-> Numero di Object Properties: " + ontologia.getObjectPropertiesInSignature().size());
                System.out.println("-> Numero di Individui (Named Individuals): " + ontologia.getIndividualsInSignature().size());
                System.out.println("\n### Esempio Classi Trovate ###");
                ontologia.getClassesInSignature().stream().forEach(owlClass -> System.out.println(" - " + owlClass.getIRI().getFragment()));
                String baseIRI = ontologia.getOntologyID().getOntologyIRI()
                    .map(iri -> iri.toString())
                    .orElse("http://www.semanticweb.org/untitled-ontology-6"); // Fallback al tuo IRI esempio

                // Assicurati che l'IRI finisca con # o /
                if (!baseIRI.endsWith("#") && !baseIRI.endsWith("/")) {
                    baseIRI += "#";
                }

                // 4. Inizializzazione del Reasoner (Openllet/Pellet per SWRL)
                // Usiamo la Factory perché il generatore di spiegazioni deve poterne creare nuove istanze
                OWLReasonerFactory reasonerFactory = OpenlletReasonerFactory.getInstance();
                OWLReasoner reasoner = reasonerFactory.createReasoner(ontologia, new SimpleConfiguration());

                // Verifica coerenza iniziale
                if (!reasoner.isConsistent()) {
                    System.err.println("❌ L'ontologia è inconsistente!");
                    return;
                }

                // 5. Definizione dell'Assioma da Spiegare
                // Vogliamo spiegare: PersonaEmergenza(luigi)
                OWLNamedIndividual luigi = df.getOWLNamedIndividual(IRI.create(baseIRI + "Luigi")); // O il nome esatto nell'ontologia
                OWLClass personaEmergenza = df.getOWLClass(IRI.create(baseIRI + "PersonaEmergenza"));

                OWLClassAssertionAxiom assiomaDaSpiegare = df.getOWLClassAssertionAxiom(personaEmergenza, luigi);

                System.out.println("\n--- Verifica Inferenza ---");
                // Controlliamo prima se il reasoner lo deduce davvero
                if (reasoner.isEntailed(assiomaDaSpiegare)) {
                    
                    System.out.println("Generazione della spiegazione in corso (potrebbe richiedere tempo)...");
        
                    // 6. Generazione della Spiegazione
                    // DefaultExplanationGenerator usa una tecnica "black box" creando istanze del reasoner
                    DefaultExplanationGenerator gen = new DefaultExplanationGenerator(
                            man, 
                            reasonerFactory, 
                            ontologia, 
                            null
                    );

                    // Otteniamo il set di assiomi che causano l'inferenza
                    Set<OWLAxiom> spiegazione = gen.getExplanation(assiomaDaSpiegare);
                    System.out.println("\n### SPIEGAZIONE (PERCHÉ LUIGI È UNA PERSONA EMERGENZA?) ###");
                    System.out.println(spiegazione);
                } else {
                    System.out.println("⚠️ Il reasoner NON deduce assiomi inerenti");           
                }
            } catch (OWLOntologyCreationException e) {
                // Gestisci l'errore nel caso in cui il caricamento fallisca
                System.err.println("❌ ERRORE: Non è stato possibile caricare l'ontologia.");
                e.printStackTrace();
            }
        } else {
            // Cosa succede se .exists() è false
            System.err.println("❌ ERRORE: Il file non è stato trovato.");
            System.err.println("Controllare il percorso assoluto: " + fileOntologia.getAbsolutePath());
        }
    }  
}