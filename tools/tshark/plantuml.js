import * as fs from 'fs';

export class SequenceGenerator {
    participants = [];
    _fd;
    filename;

    constructor(participants, fn = "sequence.puml") {
        this.participants = participants;
        this.open(fn);
    }
    /** Convenience wrapper to write a line (with newline automatically appended) to file */
    write(str) {
        if (this._fd) {
            return this._fd.write(str + '\n');
        } else {
            return null;
        }
    }
    
    open(fn) {
        if (this._fd) {
            this.close();
        }

        this._fd = fs.createWriteStream( fn );
        this.write( '@startuml' );
        this.participants.forEach( d => this.write('participant ' + d ) );  // or participant $key as $name
        this.filename = fn;
        
    }
    add(from, to, description) {
        this.write( `${from} -> ${to} : ${description}`);
    }
    startGroup(title) {
        if (this._fd) {
            this.write( `group ${title}` );
            this.activeGroup = true;
        }
    }
    endGroup() {
        this.write( `end` );
        this.activeGroup = false;
    }
    close() {
        if (this._fd) {
            if (this.activeGroup) this.endGroup();
            this.write('@enduml');
            this._fd.close();
            this._fd = null;
        }
    }
}
